`timescale 1ns/1ps
//============================================================================
//  Bad Lands inputs -- /SWITCHES (18C LS244), the LETA 20C, the two steering
//  quadrature generators and the LM556 pedal timers of schematic sheet 1.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.
//
//  Derivations, all GPL-compatible and listed in NOTICE.md:
//   * `badlands_leta` follows
//     `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_leta.sv` (GPL-3.0,
//     same author), which in turn follows the Atari System 2 core's
//     `sys2_leta.sv` (GPL-2.0-or-later, RetroShrimp).  Bad Lands wires FOUR
//     live channels, CK = /1H and RESOL = GND, so the rate, the channel count
//     and the decode factor are all different.
//   * `badlands_quad_gen` follows
//     `Arcade-AtariSystem2_MiSTer/rtl/sound/sys2_quad_gen.sv`
//     (GPL-2.0-or-later, RetroShrimp).
//   * `badlands_pedal_556` is Bad Lands' own: no other core in the tree has a
//     555 astable on an encoder input.
//
//  ============ /SWITCHES, 0xFE4001, 18C LS244 ============================
//  A non-inverting octal buffer; both /1G and /2G = /SWITCHES; the outputs
//  drive D7:0 only.  D15:8 is undriven and badlands_main fills it with
//  UNDRIVEN_DATA, which is why the program masks the word it reads.
//
//   | bit | source                       | polarity                        |
//   | D7  | /TEST                        | active low (1 K pull-up)        |
//   | D6  | VBLANK, SOS-2 pin 22, raw    | active high -- the one exception|
//   | D5  | JAM-Z tied to JAM-U          | right start/fire, active low    |
//   | D4  | JAM-22 tied to JAM-17        | left  start/fire, active low    |
//   | D3  | JAM-Y  + SW1-1               | FREEZE, active low              |
//   | D2  | JAM-21 + SW1-2               | FREEZE-STEP, active low         |
//   | D1  | JAM-X  + SW1-3               | OLD GAS PEDALS, active low      |
//   | D0  | JAM-20 + SW1-4               | OLD STEERING WHEELS, active low |
//
//  Each player's two start/fire buttons are tied together on the board, so
//  they present one logical input each.  D3:0 default to 1, all four SW1
//  positions off, which is what the manual requires for normal operation.
//
//  ============ LETA 20C ==================================================
//    13 CK    = /1H (SOS-2 pin 15) = 3.579545 MHz -- the whole input filter.
//               An encoder spun faster than this loses counts on real hardware
//               rather than reading wrong, and this model does too.
//    14 RESOL = GND -> 1x decoding
//     9 TEST  = GND
//    10 A1 / 11 A0 = 68000 A2 / A1  -> four live channels:
//         0xFE6001 ch0 P1 (left) wheel   0xFE6003 ch1 P2 (right) wheel
//         0xFE6005 ch2 pedal 1 (left)    0xFE6007 ch3 pedal 2 (right)
//    15-22 D0-D7 = the 68000's D7:0 (the low byte, odd addresses)
//
//  The counters are free-running 8-bit up/down counters: they wrap, a read
//  does not clear them, and they never saturate.  The program differences two
//  successive samples as a signed 8-bit value, so wrap-around is load bearing.
//
//  ============ steering ==================================================
//  R15 (10 ohm to GND) is installed and R36 (10 ohm to +5 V) is not, so the
//  four 3.3 K resistors are pull-downs and an unplugged wheel reads 0.
//  `badlands_quad_gen` parks at phase 0, both lines low, which is that state.
//
//  QUAD_INVERT = 0 is the shipped sense: turning right gives a positive delta,
//  the LETA counts up, the game's heading index steps up and the car rotates
//  clockwise on screen.  The wheel path has no complement, unlike the pedals.
//
//  ============ pedals ====================================================
//  One LM556 at 26A, both halves used.  RESET is never gated, THRESHOLD is
//  tied to TRIGGER, the timing cap is on that node and the only charge path is
//  a resistor back to OUTPUT: a textbook 50 %-duty free-running astable.  The
//  timer oscillates continuously and the pedal changes its frequency; it is
//  not a one-pulse-per-press device.
//
//  One output is the raw square through 10 K, the other the same square
//  through a 10 K / 1 nF low-pass, lagging by a fixed ~7 us -- a synthetic
//  quadrature pair with permanently one-sided phase, so the counter always
//  steps in one direction.  That direction is down: the raw output is the
//  LETA's CLK and the lagged copy its DIR, so DIR is low at every CLK rising
//  edge.
//
//    | pedal state | f            | LETA counts/frame at 1x |
//    | open        | 278-289 Hz   | -4.6 .. -4.8            |
//    | pressed     | 7.4-7.7 kHz  | -123 .. -128            |
//  The program ones-complements the pedal delta, which turns those into a
//  small throttle index at rest and a full one when the pedal is down.
//
//  PEDAL_IDLE_SILENT stops the idle oscillator so the LETA reads a flat 0x00,
//  as MAME's does, for simulation only.  With the program's complement in the
//  path a stopped oscillator reads FF, not 00, and the throttle comes out
//  inverted, so it must be 0 in any build that is run.
//============================================================================

//----------------------------------------------------------------------------
// badlands_quad_gen -- a MiSTer signed count delta -> the two square waves an
// optical encoder disc would produce on STEERING C / STEERING D.
//
// Why a generator and not "add the delta into the counter", which is what MAME
// does: making real edges keeps the 3.58 MHz sample rate, the decode factor
// and the direction logic in the path, so a player who spins faster than /1H
// can sample loses counts exactly as on the board.
//
// Phase arithmetic.  The Gray sequence is 00 -> 01 -> 11 -> 10 -> 00 written
// (CLK, DIR), which is "forward" = the LETA counts up.  At 2x decoding one
// count is two phase steps; at 1x (this board, RESOL = GND) one count is a
// full cycle = four phase steps.  PHASE_PER_COUNT carries that, so a requested
// delta of N always reads back as N counts whatever the decode factor -- the
// MiSTer delta is a count request, not a disc position.
//----------------------------------------------------------------------------
/* verilator lint_off DECLFILENAME */
module badlands_quad_gen #(
	parameter int PHASE_PER_COUNT = 4,   // 4 at 1x decoding, 2 at 2x
	parameter bit QUAD_INVERT     = 1'b0 // swap CLK and DIR
)(
	input  logic              clk,
	input  logic              reset,
	input  logic              ce_step,     // emit at most one phase step

	input  logic signed [8:0] delta,       // signed count delta, + = clockwise
	input  logic              delta_valid, // 1-clk pulse

	output logic              q_clk,       // -> LETA CLKn  (STEERING C)
	output logic              q_dir,       // -> LETA DIRn  (STEERING D)

	output logic signed [15:0] backlog     // phase steps still owed
);

	logic signed [15:0] phase_target, phase_now;

	wire signed [15:0] req_phase =
		delta_valid ? (16'(delta) * 16'(PHASE_PER_COUNT)) : 16'sd0;

	wire signed [15:0] remaining = phase_target + req_phase - phase_now;
	assign backlog = remaining;

	// The part-select is pulled out of the always_comb: Icarus 12 refuses a
	// constant select inside an inferred sensitivity list ("sorry: constant
	// selects in always_* processes are not currently supported").
	wire [1:0] ph = phase_now[1:0];

	logic p_clk, p_dir;
	always_comb begin
		case (ph)
			2'd0:    begin p_clk = 1'b0; p_dir = 1'b0; end
			2'd1:    begin p_clk = 1'b0; p_dir = 1'b1; end
			2'd2:    begin p_clk = 1'b1; p_dir = 1'b1; end
			default: begin p_clk = 1'b1; p_dir = 1'b0; end
		endcase
	end

	// QUAD_INVERT swaps the two wires, which is exactly what getting Q0/Q1 the
	// wrong way round on the steering PCB does.
	assign q_clk = QUAD_INVERT ? p_dir : p_clk;
	assign q_dir = QUAD_INVERT ? p_clk : p_dir;

	always_ff @(posedge clk) begin
		if (reset) begin
			// Park at phase 0 = both lines low: the pulled-down idle state
			// (R15 installed, R36 not).
			phase_target <= '0;
			phase_now    <= '0;
		end else begin
			phase_target <= phase_target + req_phase;
			if (ce_step) begin
				if      (remaining > 0) phase_now <= phase_now + 16'sd1;
				else if (remaining < 0) phase_now <= phase_now - 16'sd1;
			end
		end
	end

endmodule

//----------------------------------------------------------------------------
// badlands_pedal_556 -- one half of the LM556 at 26A plus its two RC networks.
// A free-running 50 %-duty astable whose frequency the pedal changes; DIR is
// the raw output and CLK is the same square lagging by LAG_US microseconds.
//----------------------------------------------------------------------------
module badlands_pedal_556 #(
	parameter int CLK_HZ      = 57272724,
	parameter int IDLE_HZ     = 283,     // R_ext = R26 24 K -> 278-289 Hz
	parameter int PRESSED_HZ  = 7550,    // R_ext = 0        -> 7.4-7.7 kHz
	parameter int LAG_US      = 7,       // the 10 K / 1 nF low-pass crossing
	parameter bit IDLE_SILENT = 1'b0     // 1 = the idle oscillator is stopped
)(
	input  logic clk,
	input  logic reset,
	input  logic pressed,                // the player's button

	output logic q_clk,                  // -> LETA CLKn (the lagging copy)
	output logic q_dir                   // -> LETA DIRn (the raw output)
);

	localparam int HALF_IDLE = CLK_HZ / (2 * IDLE_HZ);
	localparam int HALF_PRES = CLK_HZ / (2 * PRESSED_HZ);
	localparam int LAG_CYC   = (CLK_HZ / 1000000) * LAG_US;
	localparam int CNTW      = $clog2(HALF_IDLE + 1);
	localparam int LAGW      = $clog2(LAG_CYC + 1);

	wire [CNTW-1:0] half = pressed ? CNTW'(HALF_PRES) : CNTW'(HALF_IDLE);
	wire            run  = pressed | ~IDLE_SILENT;

	logic [CNTW-1:0] cnt;
	logic            raw;

	always_ff @(posedge clk) begin
		if (reset) begin
			cnt <= '0;
			raw <= 1'b0;
		end else if (run) begin
			if (cnt >= half - 1'b1) begin
				cnt <= '0;
				raw <= ~raw;
			end else cnt <= cnt + 1'b1;
		end
	end

	// The 10 K / 1 nF low-pass: tau = 10 us, crossing ~7 us after the edge.
	// The half-period is 65 us (pressed) to 1.7 ms (idle), so the filtered copy
	// is the same clean square wave, delayed.
	logic            lag_pending, lag_val;
	logic [LAGW-1:0] lag_cnt;
	logic            raw_d;

	always_ff @(posedge clk) begin
		if (reset) begin
			raw_d       <= 1'b0;
			lag_pending <= 1'b0;
			lag_cnt     <= '0;
			lag_val     <= 1'b0;
			q_clk       <= 1'b0;
		end else begin
			raw_d <= raw;
			if (raw != raw_d) begin
				lag_pending <= 1'b1;
				lag_val     <= raw;
				lag_cnt     <= LAGW'(LAG_CYC - 1);
			end else if (lag_pending) begin
				if (lag_cnt == 0) begin
					lag_pending <= 1'b0;
					q_clk       <= lag_val;
				end else lag_cnt <= lag_cnt - 1'b1;
			end
		end
	end

	assign q_dir = raw;

endmodule

//----------------------------------------------------------------------------
// badlands_leta -- the LETA 20C's four free-running 8-bit counters.
//----------------------------------------------------------------------------
module badlands_leta #(
	// RESOL = GND -> 1x decoding: one count per full quadrature cycle, i.e.
	// only the rising edge of CLK counts.  Set to 1 for the 2x decode the
	// Blasteroids board straps.
	parameter bit DECODE_X2 = 1'b0
)(
	input  logic       clk,
	input  logic       init_reset,   // FPGA init only -- the part has no reset pin

	input  logic       h1,           // the 1H level; CK = /1H, so the sampling
	                                 // edges are 1H's falling ones

	input  logic [3:0] enc_clk,      // CLK0..CLK3
	input  logic [3:0] enc_dir,      // DIR0..DIR3

	input  logic [1:0] a,            // 68000 A2:A1 -> LETA A1:A0
	output logic [7:0] dout,         // D7:0, combinational (the strobe is /CE)

	output logic [7:0] count0, count1, count2, count3
);

	logic [7:0] count [0:3];
	logic [3:0] clk_d;
	logic       h1_d;

	assign dout   = count[a];
	assign count0 = count[0];
	assign count1 = count[1];
	assign count2 = count[2];
	assign count3 = count[3];

	always_ff @(posedge clk) begin
		if (init_reset) begin
			count[0] <= 8'd0;
			count[1] <= 8'd0;
			count[2] <= 8'd0;
			count[3] <= 8'd0;
			// Prime the CLK history from the current inputs, not from zero: an
			// idle channel would otherwise fabricate one count on the first
			// sample.
			clk_d    <= enc_clk;
			h1_d     <= h1;
		end else begin
			h1_d <= h1;
			if (~h1 & h1_d) begin            // 1H falling = /1H rising = the CK pin
				clk_d <= enc_clk;
				for (int i = 0; i < 4; i++) begin
					if (DECODE_X2) begin
						// 2x: any transition of CLK counts
						if (enc_clk[i] != clk_d[i])
							count[i] <= (enc_clk[i] == enc_dir[i]) ? count[i] + 8'd1
							                                       : count[i] - 8'd1;
					end else begin
						// 1x: only the rising edge of CLK -- one count per full
						// quadrature cycle (RESOL = GND)
						if (enc_clk[i] & ~clk_d[i])
							count[i] <= (enc_dir[i]) ? count[i] + 8'd1
							                         : count[i] - 8'd1;
					end
				end
			end
		end
	end

endmodule
/* verilator lint_on DECLFILENAME */

//----------------------------------------------------------------------------
// badlands_inputs -- the top of the input block.
//----------------------------------------------------------------------------
module badlands_inputs #(
	parameter int  CLK_HZ            = 57272724,
	parameter bit  LETA_DECODE_X2    = 1'b0,   // RESOL = GND -> 1x
	parameter bit  QUAD_INVERT       = 1'b0,   // the steering CLK/DIR sense
	parameter int  PEDAL_IDLE_HZ     = 283,
	parameter int  PEDAL_PRESSED_HZ  = 7550,
	parameter int  PEDAL_LAG_US      = 7,
	// Simulation only: 1 stops the idle oscillator so the LETA reads the flat
	// 0x00 MAME's does.  It inverts the throttle, so it must be 0 in any build
	// that is run.
	parameter bit  PEDAL_IDLE_SILENT = 1'b0
)(
	input  logic       clk,
	input  logic       reset,          // FPGA init only (the parts have no /RES)

	input  logic       h1,             // the 1H level from badlands_sos2_sync

	// ---------------- /SWITCHES 0xFE4001 --------------------------------
	input  logic       test_n,         // /TEST, active low
	input  logic       vblank,         // SOS-2 pin 22, raw active high
	input  logic       p1_fire,        // left  start/fire, active high here
	input  logic       p2_fire,        // right start/fire, active high here
	input  logic [3:0] sw1_n,          // SW1 4:1 -> D3:D0, active low
	output logic [7:0] switches_dout,

	// ---------------- LETA 0xFE6001 + 2n --------------------------------
	input  logic [1:0] leta_a,         // 68000 A2:A1
	output logic [7:0] leta_dout,

	// ---------------- the MiSTer control sources ------------------------
	input  logic signed [8:0] wheel0_delta,   // ch 0 = P1 (left) wheel
	input  logic              wheel0_valid,
	input  logic signed [8:0] wheel1_delta,   // ch 1 = P2 (right) wheel
	input  logic              wheel1_valid,
	input  logic              pedal0,         // ch 2 = pedal 1 (left)
	input  logic              pedal1,         // ch 3 = pedal 2 (right)
	input  logic              pedal_invert,   // swap the pedal CLK/DIR pair

	// ---------------- observability -------------------------------------
	output logic [7:0] leta_count0, leta_count1, leta_count2, leta_count3
);

	// ---------------- 18C LS244 ------------------------------------------
	// Every switch bit is active low (1 K pull-up, grounded by the switch).
	// VBLANK on D6 is the one exception: it is the raw active-high SOS-2 pin.
	assign switches_dout = { test_n,        // D7
	                         vblank,        // D6  active high, raw
	                         ~p2_fire,      // D5  right
	                         ~p1_fire,      // D4  left
	                         sw1_n[3],      // D3  FREEZE
	                         sw1_n[2],      // D2  FREEZE-STEP
	                         sw1_n[1],      // D1  OLD GAS PEDALS
	                         sw1_n[0] };    // D0  OLD STEERING WHEELS

	// ---------------- the two steering quadrature generators -------------
	// One phase step per /1H sample, so every emitted edge is guaranteed to be
	// seen by the LETA; a faster request simply backs up and is delivered over
	// the following samples (or is lost, exactly as a real disc would be).
	logic h1_d;
	always_ff @(posedge clk) h1_d <= h1;
	wire ce_ck = ~h1 & h1_d;                     // /1H rising

	localparam int PHASE_PER_COUNT = LETA_DECODE_X2 ? 2 : 4;

	logic [3:0] enc_clk, enc_dir;
	/* verilator lint_off UNUSEDSIGNAL */
	logic signed [15:0] backlog0, backlog1;      // phase-backlog probes, unused here
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_quad_gen #(
		.PHASE_PER_COUNT (PHASE_PER_COUNT),
		.QUAD_INVERT     (QUAD_INVERT)
	) u_quad0 (
		.clk(clk), .reset(reset), .ce_step(ce_ck),
		.delta(wheel0_delta), .delta_valid(wheel0_valid),
		.q_clk(enc_clk[0]), .q_dir(enc_dir[0]), .backlog(backlog0)
	);

	badlands_quad_gen #(
		.PHASE_PER_COUNT (PHASE_PER_COUNT),
		.QUAD_INVERT     (QUAD_INVERT)
	) u_quad1 (
		.clk(clk), .reset(reset), .ce_step(ce_ck),
		.delta(wheel1_delta), .delta_valid(wheel1_valid),
		.q_clk(enc_clk[1]), .q_dir(enc_dir[1]), .backlog(backlog1)
	);

	// The pedal channels count down, on purpose.  The 556's raw output drives
	// the LETA's CLK pin and the low-passed copy drives DIR, so DIR is still
	// low at every CLK rising edge and `badlands_leta` steps down.  The program
	// needs that: its pedal blocks at 0x008472 / 0x0084A2 ones-complement the
	// 8-bit delta (`eori.w #$FF,D1`) before storing it, which the wheel blocks
	// do not, so an up-counting pedal would read as full throttle at rest and
	// none when pressed.  The steering pins are untouched -- `badlands_quad_gen`
	// still counts up.
	//
	// `pedal_invert` selects the up-counting pairing instead (OSD "Gas pedal
	// sense -> Inverted").
	logic ped2_clk, ped2_dir, ped3_clk, ped3_dir;
	assign enc_clk[2] = pedal_invert ? ped2_clk : ped2_dir;
	assign enc_dir[2] = pedal_invert ? ped2_dir : ped2_clk;
	assign enc_clk[3] = pedal_invert ? ped3_clk : ped3_dir;
	assign enc_dir[3] = pedal_invert ? ped3_dir : ped3_clk;

	// ---------------- the two LM556 halves -------------------------------
	// 1026A (pins 8-13) is pedal 1 (left)  -> LETA channel 2
	// 26A   (pins 1-6)  is pedal 2 (right) -> LETA channel 3
	badlands_pedal_556 #(
		.CLK_HZ(CLK_HZ), .IDLE_HZ(PEDAL_IDLE_HZ), .PRESSED_HZ(PEDAL_PRESSED_HZ),
		.LAG_US(PEDAL_LAG_US), .IDLE_SILENT(PEDAL_IDLE_SILENT)
	) u_pedal2 (
		.clk(clk), .reset(reset), .pressed(pedal0),
		.q_clk(ped2_clk), .q_dir(ped2_dir)
	);

	badlands_pedal_556 #(
		.CLK_HZ(CLK_HZ), .IDLE_HZ(PEDAL_IDLE_HZ), .PRESSED_HZ(PEDAL_PRESSED_HZ),
		.LAG_US(PEDAL_LAG_US), .IDLE_SILENT(PEDAL_IDLE_SILENT)
	) u_pedal3 (
		.clk(clk), .reset(reset), .pressed(pedal1),
		.q_clk(ped3_clk), .q_dir(ped3_dir)
	);

	// ---------------- the LETA itself ------------------------------------
	badlands_leta #(.DECODE_X2(LETA_DECODE_X2)) u_leta (
		.clk(clk), .init_reset(reset),
		.h1(h1), .enc_clk(enc_clk), .enc_dir(enc_dir),
		.a(leta_a), .dout(leta_dout),
		.count0(leta_count0), .count1(leta_count1),
		.count2(leta_count2), .count3(leta_count3)
	);

endmodule
