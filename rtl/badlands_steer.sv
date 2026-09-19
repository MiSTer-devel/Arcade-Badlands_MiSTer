`timescale 1ns/1ps
//============================================================================
//  badlands_steer -- the MiSTer controls for ONE steering wheel.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  The rate-converter idea and the "feed the
//  quadrature generator, never the counter" rule follow the Atari System 2
//  MiSTer core's `sys2_analog_spin.sv` / `sys2_analog_axis.sv`
//  (GPL-2.0-or-later, RetroShrimp); the summed sources, the eighth-step
//  sensitivity table and the clamped backlog follow our own Blasteroids core's
//  `blstroid_whirly.sv` (GPL-3.0); the arithmetic here is Bad Lands' own.
//
//  The output is the signed COUNT delta `badlands_main`'s `wheel{0,1}_delta` /
//  `wheel{0,1}_valid` port takes.  `badlands_quad_gen` inside `badlands_inputs`
//  turns it into real quadrature edges and the `badlands_leta` model counts
//  them at the board's own /1H sample rate.  Nothing here writes a counter, so
//  a player who spins faster than /1H can sample loses counts as on the board.
//
//  Sources, ALL SUMMED -- each is idle when its device is absent, so any of
//  them steers with no setup and there is no source to choose:
//
//    ANALOG STICK  a position, so it needs a POSITION -> RATE converter:
//                  deflection past the deadzone accumulates and emits one count
//                  per 2^ACC overflow.  The D-pad merges into the same axis at
//                  full scale, so a digital pad steers too.
//    PADDLE        an absolute 8-bit shaft position, differenced.
//    SPINNER       MiSTer's {toggle, signed delta} pair: already a delta.
//    MOUSE X       hps_io `ps2_mouse` {toggle [24], sign [4], dx [15:8]}.  Many
//                  USB spinners enumerate as a mouse rather than as MiSTer's
//                  `spinner_N`.  There is one mouse, so the top level gives it
//                  to P1 and ties `mouse_stb` low on the other wheel.
//
//  `sens_sel` (Analog stick rate) scales the stick converter only -- the ladder
//  below.  `enc_sens` (Spinner sensitivity) scales the paddle, spinner and
//  mouse in EIGHTH counts, 0.25x .. 4x, in Blasteroids' table and OSD order;
//  index 0 is 1x, the exact identity, so a cleared status word is the default
//  and one encoder count is one of the board's.  A MiSTer spinner's counts per
//  turn have nothing to do with Atari's disc, so this is a feel control, not a
//  model parameter.
//
//  The paced backlog: the program differences the 8-bit LETA once per frame as
//  a SIGNED value, so 128 or more counts inside one frame wrap and read as a
//  turn the other way -- and a mouse at 4x can ask for thousands.  Everything
//  therefore goes into a backlog CLAMPED to +/-PEND_MAX counts and drained one
//  count every PACE ce_ticks:
//
//      3 579 545 / 1024 = 3496 counts/s = 58.3 counts per 59.93 Hz frame
//
//  well under the 128 wrap and far above the game's own clamp below, so the
//  pacer only prevents the wrap; it never limits the turn rate the game can
//  use.  At +/-127 the most a flick leaves behind is ~2.2 frames of drain.
//
//  The stick ladder is anchored to that clamp: the program limits the per-frame
//  wheel delta to +/-4 LETA counts (`$FFF274`/`$FFF276`) and needs 8
//  accumulated counts per 1/32 heading step (`$FFF270`/`$FFF272`), so a full
//  32-step lock takes at least 64 frames ~ 1.07 s however fast the wheel is
//  turned.  Anything above 4 counts/frame is thrown away, so the ladder keeps
//  most of the stick's travel below it.  With `ce_tick` = ce_1h = 3.579545 MHz
//  (the LETA's own CK) and an axis of +/-127:
//
//      ACC bits 19  High     ~867 counts/s   14.5 per frame   clamps at 27.6 %
//               20  Normal   ~434             7.2             clamps at 55.3 %  (default)
//               21  Low      ~217             3.6             never clamps
//               22  Lowest   ~108             1.8             never clamps
//
//  Low and Lowest are the fine-control options.  A springy thumbstick has no
//  physical equivalent on this cabinet, so the mapping is a choice -- but one
//  anchored to a measured property of the game.
//============================================================================

module badlands_steer #(
	// axis units, against a +/-127 stick.  Rest jitter on a worn pad only;
	// the same allowance the System 2 core ships.
	parameter int DEADZONE = 12,
	// ce_ticks per emitted count, and the backlog clamp (see the header).
	parameter int PACE     = 1024,
	parameter int PEND_MAX = 127
)(
	input  logic              clk,
	input  logic              reset,
	input  logic              ce_tick,      // ce_1h -- the LETA's CK

	input  logic        [1:0] sens_sel,     // stick: 0 Normal, 1 High, 2 Low, 3 Lowest
	input  logic        [3:0] enc_sens,     // encoders: OSD index, 0 = 1x

	input  logic signed [7:0] stick_x,      // MiSTer analog X, centre 0
	input  logic        [7:0] paddle,       // MiSTer paddle, absolute position
	input  logic        [8:0] spinner,      // {toggle, signed 8-bit delta}
	input  logic              mouse_stb,    // ps2_mouse[24], toggles per packet
	input  logic signed [8:0] mouse_dx,     // {ps2_mouse[4], ps2_mouse[15:8]}
	input  logic              dig_left,
	input  logic              dig_right,

	output logic signed [8:0] delta,        // signed COUNT delta, always +/-1
	output logic              delta_valid   // 1 clk_sys pulse
);

	// =====================================================================
	//  the merged axis: the stick wins while it is live, else the D-pad
	// =====================================================================
	// "Live" is a deadzone test, not a non-zero test, so a pad that never
	// returns exactly to centre cannot creep.  The digital ends land on FULL
	// scale, so a D-pad player gets the same rate as full deflection.
	wire signed [8:0] stick_ext = {stick_x[7], stick_x};
	wire signed [8:0] stick_abs = stick_ext[8] ? -stick_ext : stick_ext;
	wire              stick_live = (stick_abs > $signed(9'(DEADZONE)));

	logic signed [8:0] axis;
	always_comb begin
		if (stick_live)     axis =  stick_ext;
		else if (dig_right) axis =  9'sd127;
		else if (dig_left)  axis = -9'sd128;
		else                axis =  9'sd0;
	end

	wire signed [8:0] axis_abs = axis[8] ? -axis : axis;

	// =====================================================================
	//  ANALOG STICK: position -> rate
	// =====================================================================
	// One accumulator; `acc_bits` selects which bit is the overflow, so
	// changing the option changes the rate and nothing else.  |axis| <= 128, so
	// the sum can never skip past the threshold by more than one count and the
	// subtract below cannot underflow.  ACCW must exceed the largest `acc_bits`
	// (22, Lowest): the threshold is 1 << acc_bits and `acc_sum` must not wrap.
	localparam int ACCW = 24;

	// The rate ladder from the header.
	wire [4:0] acc_bits = (sens_sel == 2'd1) ? 5'd19 :   // High
	                      (sens_sel == 2'd2) ? 5'd21 :   // Low
	                      (sens_sel == 2'd3) ? 5'd22 :   // Lowest
	                                           5'd20;    // Normal

	wire [ACCW-1:0] acc_thr = {{(ACCW-1){1'b0}}, 1'b1} << acc_bits;

	logic [ACCW-1:0] acc;
	logic            ana_step;      // one count is due
	logic            ana_dir;       // 1 = negative

	wire [ACCW-1:0] acc_inc = {{(ACCW-9){1'b0}}, axis_abs[8:0]};
	wire [ACCW-1:0] acc_sum = acc + acc_inc;

	always_ff @(posedge clk) begin
		ana_step <= 1'b0;
		if (reset) begin
			acc      <= {ACCW{1'b0}};
			ana_dir  <= 1'b0;
		end else if (ce_tick) begin
			// A change of direction restarts the accumulation: carrying the
			// old side's charge across zero would make the first count after
			// a reversal arrive early and feel like slop.
			if (axis_abs == 9'sd0) begin
				acc <= {ACCW{1'b0}};
			end else if (ana_dir != axis[8]) begin
				ana_dir <= axis[8];
				acc     <= {ACCW{1'b0}};
			end else if (acc_sum >= acc_thr) begin
				acc      <= acc_sum - acc_thr;
				ana_step <= 1'b1;
			end else begin
				acc <= acc_sum;
			end
		end
	end

	// =====================================================================
	//  PADDLE: an absolute position, differenced
	// =====================================================================
	// The paddle wraps at 0x00/0xFF like the shaft it stands for, so the
	// difference is taken in 8-bit modular arithmetic and then sign extended:
	// 0xFE -> 0x02 is +4, not -252.  The first change only latches
	// (`pad_seen`): hps_io's paddle reads 0 until a paddle reports, and one
	// resting at 0x80 must not arrive as a half-turn.
	logic [7:0] paddle_q;
	logic       pad_seen;
	wire  [7:0] pad_diff = paddle - paddle_q;
	logic signed [8:0] pad_d;
	logic              pad_new;
	always_ff @(posedge clk) begin
		pad_new <= 1'b0;
		if (reset) begin
			paddle_q <= 8'h00;
			pad_seen <= 1'b0;
			pad_d    <= 9'sd0;
		end else if (ce_tick && (paddle != paddle_q)) begin
			pad_d    <= $signed({pad_diff[7], pad_diff});
			paddle_q <= paddle;
			pad_seen <= 1'b1;
			pad_new  <= pad_seen;
		end
	end

	// =====================================================================
	//  SPINNER: MiSTer's {toggle, signed delta}
	// =====================================================================
	logic              spin_tog_q;
	logic signed [8:0] spin_d;
	logic              spin_new;
	always_ff @(posedge clk) begin
		spin_new <= 1'b0;
		if (reset) begin
			spin_tog_q <= 1'b0;
			spin_d     <= 9'sd0;
		end else if (ce_tick && (spinner[8] != spin_tog_q)) begin
			spin_tog_q <= spinner[8];
			spin_d     <= $signed({spinner[7], spinner[7:0]});
			spin_new   <= 1'b1;
		end
	end

	// =====================================================================
	//  MOUSE X: hps_io's {toggle, signed 9-bit dx}
	// =====================================================================
	// hps_io flips ps2_mouse[24] when the packet command ENDS, after the three
	// data bytes are stored, so the dx read on the toggle change is the new one.
	logic              mouse_tog_q;
	logic signed [8:0] mouse_d;
	logic              mouse_new;
	always_ff @(posedge clk) begin
		mouse_new <= 1'b0;
		if (reset) begin
			mouse_tog_q <= 1'b0;
			mouse_d     <= 9'sd0;
		end else if (ce_tick && (mouse_stb != mouse_tog_q)) begin
			mouse_tog_q <= mouse_stb;
			mouse_d     <= mouse_dx;
			mouse_new   <= 1'b1;
		end
	end

	// =====================================================================
	//  the ENCODER sum, scaled in eighths with a carried residue
	// =====================================================================
	// All three latch on the same ce_tick, so their `_new` pulses can coincide;
	// |enc_raw| <= 128 + 128 + 256 = 512.
	wire signed [10:0] enc_raw =
		(pad_new   ? 11'(pad_d)   : 11'sd0) +
		(spin_new  ? 11'(spin_d)  : 11'sd0) +
		(mouse_new ? 11'(mouse_d) : 11'sd0);

	// Blasteroids' table and OSD order (Arcade-Blasteroids.sv CONF_STR).
	logic [5:0] mul8;
	always_comb begin
		case (enc_sens)
			4'd0:  mul8 = 6'd8;    // 1x   (default)
			4'd1:  mul8 = 6'd9;    // 1.125x
			4'd2:  mul8 = 6'd10;   // 1.25x
			4'd3:  mul8 = 6'd11;   // 1.375x
			4'd4:  mul8 = 6'd12;   // 1.5x
			4'd5:  mul8 = 6'd14;   // 1.75x
			4'd6:  mul8 = 6'd16;   // 2x
			4'd7:  mul8 = 6'd20;   // 2.5x
			4'd8:  mul8 = 6'd24;   // 3x
			4'd9:  mul8 = 6'd32;   // 4x
			4'd10: mul8 = 6'd2;    // 0.25x
			4'd11: mul8 = 6'd3;    // 0.375x
			4'd12: mul8 = 6'd4;    // 0.5x
			4'd13: mul8 = 6'd5;    // 0.625x
			4'd14: mul8 = 6'd6;    // 0.75x
			default: mul8 = 6'd7;  // 0.875x
		endcase
	end

	// The raw sum is registered first, so the multiply is the only logic
	// between two flops; one value goes in and one comes out per clock, so
	// nothing is lost or counted twice.
	logic signed [10:0] enc_q;
	always_ff @(posedge clk) begin
		if (reset) enc_q <= '0;
		else       enc_q <= enc_raw;
	end

	// |enc_q| <= 512 and mul8 <= 32, so |scaled_q| <= 16 391: fits 16 bits.
	logic signed [15:0] frac;
	wire  signed [15:0] mul8_s   = $signed({10'd0, mul8});
	wire  signed [15:0] scaled_q = (16'(enc_q) * mul8_s) + frac;
	// Arithmetic shift = floor, so the residue is always 0..7 and is carried in
	// `frac`; a one-count nudge below 1x then arrives instead of rounding away.
	wire  signed [15:0] whole    = scaled_q >>> 3;

	// |whole| <= 2049; the backlog clamps anyway, so cap it at +/-1023 to keep
	// the backlog adder narrow.
	logic signed [10:0] enc_s;
	always_ff @(posedge clk) begin
		if (reset) begin
			frac  <= '0;
			enc_s <= '0;
		end else begin
			frac  <= scaled_q - (whole <<< 3);
			enc_s <= (whole >  16'sd1023) ? 11'sd1023  :
			         (whole < -16'sd1023) ? -11'sd1023 : whole[10:0];
		end
	end

	// =====================================================================
	//  the paced, clamped backlog
	// =====================================================================
	localparam int PW = $clog2(PACE);
	localparam logic signed [12:0] PMAX   = 13'(PEND_MAX);
	localparam logic signed [10:0] PMAX11 = 11'(PEND_MAX);

	logic [PW-1:0]      pace_cnt;
	logic signed [10:0] pend;

	wire pace_tick = ce_tick && (pace_cnt == PW'(PACE - 1));
	wire emit      = pace_tick && (pend != 11'sd0);

	wire signed [10:0] stick_d = ana_step ? (ana_dir ? -11'sd1 : 11'sd1) : 11'sd0;
	wire signed [10:0] drain   = emit ? (pend[10] ? 11'sd1 : -11'sd1) : 11'sd0;
	wire signed [12:0] pend_nx = 13'(pend) + 13'(enc_s) + 13'(stick_d) + 13'(drain);

	always_ff @(posedge clk) begin
		if (reset) begin
			pace_cnt <= '0;
			pend     <= '0;
		end else begin
			if (ce_tick) pace_cnt <= pace_cnt + 1'b1;
			if      (pend_nx >  PMAX) pend <=  PMAX11;
			else if (pend_nx < -PMAX) pend <= -PMAX11;
			else                      pend <= pend_nx[10:0];
		end
	end

	assign delta       = pend[10] ? -9'sd1 : 9'sd1;
	assign delta_valid = emit;

endmodule
