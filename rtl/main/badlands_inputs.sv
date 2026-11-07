`timescale 1ns/1ps
//============================================================================
//  Bad Lands inputs -- /SWITCHES (18C LS244), the LETA 20C, the two steering
//  quadrature generators and the LM556 pedal timers of schematic sheet 1.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.
//
//  Derivations, all GPL-compatible and listed in NOTICE.md:
//   * `badlands_leta` implements the behaviour of JROK's LETA_REP.vhd ("LETA
//     on a CPLD", jrok.com/project/leta: a replacement built by bench-testing
//     a real 137304-2002), as transcribed by the Atari System 2 MiSTer core's
//     `rtl/sound/sys2_leta.sv` (GPL-2.0-or-later, RetroShrimp; MiSTer-devel
//     Arcade-AtariSystem2_MiSTer commit f35c810b) and by our own Rampart
//     core's `rampart_leta.sv` (GPL-3.0, same author).  No VHDL is copied.
//     Bad Lands wires FOUR live channels, CK = /1H, RESOL = GND, TEST = GND.
//   * `badlands_quad_gen` follows the same System 2 core's
//     `rtl/sound/sys2_quad_gen.sv` (GPL-2.0-or-later, RetroShrimp).
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
//    13 CK    = /1H (SOS-2 pin 15) = 3.579545 MHz, the sample clock
//    14 RESOL = GND -> 2x: DIR edges count, CLK edges do not
//     9 TEST  = GND
//    10 A1 / 11 A0 = 68000 A2 / A1  -> four live channels:
//         0xFE6001 ch0 P1 (left) wheel   0xFE6003 ch1 P2 (right) wheel
//         0xFE6005 ch2 pedal 1 (left)    0xFE6007 ch3 pedal 2 (right)
//    15-22 D0-D7 = the 68000's D7:0 (the low byte, odd addresses)
//
//  Per channel, on every CK rising edge (JROK's bench-tested behaviour):
//   * CLK and DIR are each shifted into a 3-sample history; a level is
//     SETTLED only when three consecutive samples agree, so 1- and 2-sample
//     glitches are ignored.  A state must last >= 3 CK periods (0.84 us).
//   * the settled (CLK, DIR) is compared with the previous settled pair:
//       DIR changed, CLK not: always counts; UP iff new DIR == CLK
//       CLK changed, DIR not: counts only at RESOL = 1; UP iff new CLK != DIR
//       both changed:         no count
//   * RESOL = 0 is therefore 2x and RESOL = 1 is 4x.  There is NO 1x mode.
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
//  `badlands_quad_gen` makes CLK lead DIR, the LETA counts up, the game's
//  heading index steps up and the car rotates clockwise on screen.  The wheel
//  path has no complement, unlike the pedals.
//
//  ============ pedals ====================================================
//  One LM556 at 26A, both halves used.  RESET is never gated, THRESHOLD is
//  tied to TRIGGER, the timing cap is on that node and the only charge path is
//  a resistor back to OUTPUT: a textbook 50 %-duty free-running astable.  The
//  timer oscillates continuously and the pedal changes its frequency; it is
//  not a one-pulse-per-press device.
//
//  The raw output goes through R43 10 K to the LETA's DIR; the same square
//  through R41 10 K / C39 1 nF goes to its CLK, lagging by ~7 us -- a
//  synthetic quadrature pair whose phase is permanently one-sided.  DIR
//  leads, so at every DIR edge CLK still holds the OTHER level and both DIR
//  edges count DOWN (two counts per cycle at RESOL = GND).
//
//  Rates, DERIVED by integrating the network with a bipolar output stage
//  (V_OH ~ Vcc - 1.4 V, R42 pulling the node higher where the load allows;
//  tests/pedal_556_rate.py):
//    | pedal state | f (nominal, band)          | LETA counts/frame at 2x |
//    | open        | 271 Hz   (229 .. 324 Hz)   |   -9.0  (-7.6 .. -10.8) |
//    | pressed     | 4.94 kHz (4.13 .. 6.06 kHz)| -165    (-138 .. -202)  |
//  Every corner stays below 256 counts a frame, so the 8-bit delta never
//  wraps.  The program ones-complements the pedal delta, which turns those
//  into a small throttle input at rest and a large one when the pedal is down;
//  its quantiser auto-ranges, so the absolute rate does not matter.
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
// does: making real edges keeps the 3.58 MHz sample rate, the input filter,
// the decode factor and the direction logic in the path, so a player who spins
// faster than /1H can sample loses counts exactly as on the board.
//
// Phase arithmetic.  "Forward" (a positive request) is the Gray sequence
// 00 -> 10 -> 11 -> 01 -> 00 written (CLK, DIR): CLK LEADS.  Each DIR edge
// then lands with DIR == CLK, which the LETA counts UP; the CLK edges in
// between are not counted at RESOL = GND.  So at 2x one count is two phase
// steps (PHASE_PER_COUNT = 2; 1 at 4x), and a requested delta of N reads back
// as N counts -- the MiSTer delta is a count request, not a disc position.
//
// Hold.  The LETA only accepts a level after three identical CK samples, so
// each state is held for HOLD_TICKS ce_step ticks (ce_step is the same /1H
// edge the LETA samples on).  At the default 4 that is 895 k phase steps a
// second, ~447 k counts a second at 2x -- thousands of times what the pacer
// in badlands_steer ever asks for.
//
// The sequence and the LETA's sign move TOGETHER: reverse one without the
// other and the wheel steers backwards.
//----------------------------------------------------------------------------
/* verilator lint_off DECLFILENAME */
module badlands_quad_gen #(
	parameter int PHASE_PER_COUNT = 2,   // 2 at 2x decoding (RESOL = 0), 1 at 4x
	parameter int HOLD_TICKS      = 4,   // ce_step ticks per state, 3..8
	parameter bit QUAD_INVERT     = 1'b0 // swap CLK and DIR
)(
	input  logic              clk,
	input  logic              reset,
	input  logic              ce_step,     // one LETA sample (/1H rising)

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
			2'd1:    begin p_clk = 1'b1; p_dir = 1'b0; end
			2'd2:    begin p_clk = 1'b1; p_dir = 1'b1; end
			default: begin p_clk = 1'b0; p_dir = 1'b1; end
		endcase
	end

	// QUAD_INVERT swaps the two wires, which is exactly what getting Q0/Q1 the
	// wrong way round on the steering PCB does.
	assign q_clk = QUAD_INVERT ? p_dir : p_clk;
	assign q_dir = QUAD_INVERT ? p_clk : p_dir;

	// Hold divider: a step is allowed on one ce_step in HOLD_TICKS.  A fixed
	// 3-bit counter and a literal-width cast (Quartus 17.0 rejects casts it
	// cannot size at parse time).
	localparam logic [2:0] HOLD_LAST = 3'(HOLD_TICKS - 1);
	logic [2:0] hold_cnt;
	wire step_ok = ce_step && (hold_cnt == HOLD_LAST);

	always_ff @(posedge clk) begin
		if (reset) begin
			// Park at phase 0 = both lines low: the pulled-down idle state
			// (R15 installed, R36 not).
			phase_target <= '0;
			phase_now    <= '0;
			hold_cnt     <= 3'd0;
		end else begin
			if (ce_step) hold_cnt <= (hold_cnt == HOLD_LAST) ? 3'd0 : hold_cnt + 3'd1;
			phase_target <= phase_target + req_phase;
			if (step_ok) begin
				if      (remaining > 0) phase_now <= phase_now + 16'sd1;
				else if (remaining < 0) phase_now <= phase_now - 16'sd1;
			end
		end
	end

endmodule

//----------------------------------------------------------------------------
// badlands_pedal_556 -- one half of the LM556 at 26A plus its two RC networks.
// A free-running 50 %-duty astable whose frequency the pedal changes; q_dir is
// the raw output (R43 -> LETA DIR) and q_clk the same square lagging by LAG_US
// microseconds (R41 + C39 -> LETA CLK).  The rates are DERIVED, not measured.
//----------------------------------------------------------------------------
module badlands_pedal_556 #(
	parameter int CLK_HZ      = 57272724,
	parameter int IDLE_HZ     = 271,     // R_ext = R26 24 K -> 229-324 Hz
	parameter int PRESSED_HZ  = 4940,    // R_ext = 0        -> 4.13-6.06 kHz
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

	// The 10 K / 1 nF low-pass: tau = 10 us, crossing ~4-12 us after the edge
	// depending on the edge and the input threshold; 7 us is used.  The
	// half-period is ~100 us (pressed) to ~1.8 ms (idle), so the filtered copy
	// is the same clean square wave, delayed.  The exact lag cannot change a
	// count: it only has to exceed the LETA's 3-sample filter (0.84 us) and
	// stay well inside a half-period.
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
// badlands_leta -- the LETA 20C's four free-running 8-bit counters, with the
// chip's 3-sample input filter and its edge rule (see the file header).
//----------------------------------------------------------------------------
module badlands_leta #(
	// RESOL (pin 14).  0 = 2x: DIR edges only -- this board ties it to GND.
	// 1 = 4x: DIR and CLK edges.  TEST (pin 9) is tied to GND and not modelled.
	parameter bit RESOL = 1'b0
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
	logic [2:0] hc [0:3];            // last three CLK samples, newest in bit 0
	logic [2:0] hd [0:3];            // last three DIR samples
	logic [3:0] lc, ld;              // last SETTLED CLK / DIR level
	logic       h1_d;

	assign dout   = count[a];
	assign count0 = count[0];
	assign count1 = count[1];
	assign count2 = count[2];
	assign count3 = count[3];

	always_ff @(posedge clk) begin
		if (init_reset) begin
			for (int i = 0; i < 4; i++) begin
				count[i] <= 8'd0;
				// Prime the filter and the settled levels from the CURRENT
				// inputs, not from zero: an input sitting high would otherwise
				// read as a fresh 0 -> 1 and fabricate a count.
				hc[i]    <= {3{enc_clk[i]}};
				hd[i]    <= {3{enc_dir[i]}};
			end
			lc   <= enc_clk;
			ld   <= enc_dir;
			h1_d <= h1;
		end else begin
			h1_d <= h1;
			if (~h1 & h1_d) begin            // 1H falling = /1H rising = the CK pin
				for (int i = 0; i < 4; i++) begin
					hc[i] <= {hc[i][1:0], enc_clk[i]};
					hd[i] <= {hd[i][1:0], enc_dir[i]};

					// Everything below reads the history as it stood BEFORE
					// this sample.
					if (hc[i] == 3'b000) lc[i] <= 1'b0;
					if (hc[i] == 3'b111) lc[i] <= 1'b1;
					if (hd[i] == 3'b000) ld[i] <= 1'b0;
					if (hd[i] == 3'b111) ld[i] <= 1'b1;

					if ((hc[i] == 3'b000 || hc[i] == 3'b111) &&
					    (hd[i] == 3'b000 || hd[i] == 3'b111)) begin
						// DIR edge, CLK steady: counted at either resolution
						if (hc[i][0] == lc[i] && hd[i][0] != ld[i])
							count[i] <= (hd[i][0] == hc[i][0]) ? count[i] + 8'd1
							                                   : count[i] - 8'd1;
						// CLK edge, DIR steady: counted only at RESOL = 1
						else if (RESOL && hc[i][0] != lc[i] && hd[i][0] == ld[i])
							count[i] <= (hc[i][0] != hd[i][0]) ? count[i] + 8'd1
							                                   : count[i] - 8'd1;
						// both changed: no count
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
	parameter bit  LETA_RESOL        = 1'b0,   // RESOL = GND -> 2x
	parameter bit  QUAD_INVERT       = 1'b0,   // the steering CLK/DIR sense
	parameter int  PEDAL_IDLE_HZ     = 271,
	parameter int  PEDAL_PRESSED_HZ  = 4940,
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
	// At most one phase step per HOLD_TICKS /1H samples, so every emitted state
	// lasts long enough for the LETA's 3-sample filter; a faster request simply
	// backs up and is delivered over the following samples.
	logic h1_d;
	always_ff @(posedge clk) h1_d <= h1;
	wire ce_ck = ~h1 & h1_d;                     // /1H rising

	localparam int PHASE_PER_COUNT = LETA_RESOL ? 1 : 2;

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

	// The pedal channels count down, on purpose.  Schematic sheet 1: the 556's
	// raw output reaches the LETA's DIR (R43) and the low-passed copy its CLK
	// (R41 + C39), so DIR leads and both DIR edges count DOWN.  The program
	// needs that: its pedal blocks at 0x008472 / 0x0084A2 ones-complement the
	// 8-bit delta (`eori.w #$FF,D1`) before storing it, which the wheel blocks
	// do not, so an up-counting pedal would read as full throttle at rest and
	// none when pressed.
	//
	// `pedal_invert` selects the other pairing (raw -> CLK, lagged -> DIR),
	// which counts UP (OSD "Gas pedal sense -> Inverted").
	logic ped2_clk, ped2_dir, ped3_clk, ped3_dir;
	assign enc_clk[2] = pedal_invert ? ped2_dir : ped2_clk;
	assign enc_dir[2] = pedal_invert ? ped2_clk : ped2_dir;
	assign enc_clk[3] = pedal_invert ? ped3_dir : ped3_clk;
	assign enc_dir[3] = pedal_invert ? ped3_clk : ped3_dir;

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
	badlands_leta #(.RESOL(LETA_RESOL)) u_leta (
		.clk(clk), .init_reset(reset),
		.h1(h1), .enc_clk(enc_clk), .enc_dir(enc_dir),
		.a(leta_a), .dout(leta_dout),
		.count0(leta_count0), .count1(leta_count1),
		.count2(leta_count2), .count3(leta_count3)
	);

endmodule
