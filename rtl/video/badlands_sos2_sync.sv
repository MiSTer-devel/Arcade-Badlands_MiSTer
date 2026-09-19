`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the SOS-2 sync generator (Atari 137550-001 at 25M, schematic
//  047194-01 sheet 1 grid 25M).
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE and NOTICE.md.  Structurally ported from the Blasteroids MiSTer
//  core's `core/rtl/video/blstroid_sos2_sync.sv` (GPL-3.0, same author), which
//  models the same Atari custom on the same 14.318181 MHz crystal; the raster
//  positions are Bad Lands' own.
//
//  There is no horizontal decode here.  The SOS-2's own HBLANK (pin 21) and
//  /HSYNC (pin 23) outputs are unconnected on this board: horizontal blanking
//  and sync are regenerated in GAL 26R (136074-2000) and the sync is delayed
//  through 13V, so they live in badlands_hstrobes.sv, and composite sync (8J
//  F00 + 6F 7406) in badlands_video.sv.  H_TOTAL stays here because the H
//  counter itself is inside the custom.
//
//  ---- horizontal ---------------------------------------------------------
//  H_TOTAL = 456 counts of 7M follows from GAL 26R alone: its /FIRSTH
//  product-term set {453,454,455,0,1,2,3,4} is contiguous only if 455 wraps to
//  0.  Independently /BLANK yields exactly 336 active counts, which is the
//  published visible width.
//
//      7M    = 14.318181 / 2 = 7.1590905 MHz    period 139.68 ns
//      line  = 456 x 139.68 ns = 63.696 us      line rate 15.7006 kHz
//
//  7M is both the pixel clock and the H-counter clock, so there is one pixel
//  per H count — unlike Blasteroids, which counts two.
//
//  ---- vertical -----------------------------------------------------------
//   * V_TOTAL = 262 lines, i.e. 59.93 Hz, which matches a frequency-counter
//     reading taken from a real board.
//   * VCLK (pin 39) = /256H.  256H is high for H = 256..455, so /256H rises at
//     the 455 -> 0 wrap and the V counter advances once per line at H = 0.
//   * 256V (pin 37) is unconnected, so the counter is at most 9 bits.
//   * V = 0 is the first displayed line and 240 lines are visible, so VBLANK is
//     high for V = 240..261.
//   * VBLANK (pin 22) is an active-high output, taken raw to /SWITCHES D6, the
//     watchdog clock, GAL 26R pin 12 and the sound IRQ.
//   * /VSYNC (pin 24) drives 8J/6F and is low for five lines, V = 243..247.
//   * /VRES (pin 38) goes only to VMATCH 17U pin 28, and is low over the last
//     line of the frame — the only placement consistent with VMATCH keeping its
//     own copy of the line counter in step with 1V..128V.
//
//  ---- the 7M / 14M phase -------------------------------------------------
//  7M is high over the first half of an H count and low over the second half;
//  it rises on the count boundary.  14M is high over the first half of each of
//  the two 14M periods inside a count.  Two independent facts fix this: GAL 26R
//  is clocked by 7M rising and its registered /FIRSTH, /BLANK and /8HD6H sets
//  are the solved sums shifted late by exactly one 7M period, which only holds
//  if the SOS-2's H counter also advances on 7M rising; and the colour RAM's A0
//  is 7M with 10K latching the index at /7M rising = 7M falling, which fits a
//  whole 16-bit colour inside one count only if 7M falls mid-count.
//
//  With `badlands_core`'s enables (`ce_7m` marks the last clk_sys cycle of the
//  count `h` currently names, so the edge that ends that cycle makes it h+1),
//  that is `m7 = ~ph[2]`, `m14 = ~ph[1]` over `ph = 0..7`.  Everything
//  downstream — GAL 26R's registers, GAL 21R, 18R's two decisions per pixel,
//  7N/10K on /7M, 3K on 7M, PIXCLK = NOR(14E./Q, 7M) — is written against this
//  convention.
//
//  ---- clocking -----------------------------------------------------------
//      ce_14m  = clk_sys/4   the 14M colour-RAM byte rate
//      ce_7m   = clk_sys/8   one pixel and one 1H count; a strict subset of
//                            ce_14m, so `ce_7m` implies `ce_14m`.
//  Markers (`line_start`, ...) are one clk_sys cycle wide and qualified by
//  `ce_7m`, the same convention the Xybots / Blasteroids / Vindicators cores
//  use.
//============================================================================

module badlands_sos2_sync #(
	// ---- horizontal -------------------------------------------------------
	parameter int H_TOTAL  = 456,   // counts of 7M per line = 456 pixels
	// ---- vertical ---------------------------------------------------------
	parameter int V_TOTAL  = 262,   // lines per frame; 59.93 Hz
	parameter int VB_START = 240,   // 240 visible lines
	parameter int VS_START = 243,   // /VSYNC is low over V = 243..247,
	parameter int VS_END   = 248    // i.e. five lines
)(
	input  logic       clk,        // clk_sys, 57.272724 MHz
	input  logic       reset,      // synchronous, active high
	input  logic       ce_14m,     // clk_sys/4 — the 14M byte rate
	input  logic       ce_7m,      // clk_sys/8 — one PIXEL = one `1H` COUNT

	// ---- clock tree (levels, never use one of these as a clock) ----------
	output logic       m14,        // 14M   (high over the first half of each
	output logic       m14_n,      // /14M   14M period — see the header)
	output logic       m7,         // 7M    (high over the first half of a count,
	output logic       m7_n,       // /7M    rising on the count boundary)
	output logic [2:0] ph,         // position 0..7 inside one count

	// ---- the 23R / 18R line-buffer clocks --------------------------------
	// Not SOS-2 pins: 23R F74 half 1 (D = 7M, CK = /14MP, /Q = MODCLK) and
	// 18R F74 half 1 (D = 7M, CK = 14M, /PRE = MODCLK, Q = MORCLK).  They live
	// here because they are pure functions of the 7M/14M phase this module
	// defines, and because MODCLK also leaves the video block (17J F32 makes
	// /RAMWP from it).
	output logic       modclk,     // 50 % duty, high over ph 6,7,0,1
	output logic       morclk,     // 75 % duty, low over ph 0,1 only

	// ---- exact clock-edge enables ----------------------------------------
	// Same convention as `ce_7m`: each is high for the one clk_sys cycle whose
	// ending posedge IS that clock edge, so a register written under it lands
	// on the real edge.  Deriving these from the levels with a
	// register-and-compare would put every edge one clk_sys cycle late, which
	// inverts the two-phase 7M protocol in badlands_prio — the whole video
	// pipeline hangs on getting this right, so it is published once, here.
	output logic       ce_m7_rise,   // 7M rising  = the H count boundary (ph 7)
	output logic       ce_m7_fall,   // 7M falling = mid-count            (ph 3)
	output logic       ce_m14_rise,  // 14M rising, twice a count       (ph 3, 7)
	output logic       ce_modclk_r,  // MODCLK rising, LB DCLK            (ph 5)
	output logic       ce_morclk_r,  // MORCLK rising, LB RCLK            (ph 1)

	// ---- counters --------------------------------------------------------
	output logic [8:0] h,          // 1H..256H
	output logic [8:0] v,          // 1V..256V (the true 0..V_TOTAL-1 line index)

	// ---- the individual counter taps every later sheet consumes ----------
	// Named exactly as the schematic nets, so sheets 1-6 can be transcribed
	// literally without re-deriving bit positions.
	output logic       h1,   output logic h1_n,     //   1H  /1H
	output logic       h2,   output logic h2_n,     //   2H  /2H
	output logic       h4,   output logic h4_n,     //   4H  /4H
	output logic       h8,   output logic h16,      //   8H  16H
	output logic       h32,  output logic h64,      //  32H  64H
	output logic       h128, output logic h256,     // 128H 256H
	output logic       h256_n,                      // /256H -> SOS-2 VCLK
	output logic       v1,   output logic v1_n,     //   1V  /1V
	output logic       v2,   output logic v4,       //   2V   4V
	output logic       v8,   output logic v16,      //   8V  16V
	output logic       v32,  output logic v64,      //  32V  64V   (32V -> snd IRQ)
	output logic       v128, output logic v256,     // 128V 256V

	// ---- the two delayed 4H copies (SOS-2 pins 2 and 19) -----------------
	output logic       h_4hd1h,    // 4HD1H — 4H delayed one count  (-> /4HD1H,
	                               //         22K LS374 CK)
	output logic       h_4hd1h_n,  // /4HD1H (26U F04 pins 11 -> 10)
	output logic       h_4hd2h,    // 4HD2H — 4H delayed two counts (-> 11V CK,
	                               //         MO ROM A1 via /4HD2H)
	output logic       h_4hd2h_n,  // /4HD2H (26U F04 pins 3 -> 4)

	// ---- vertical blanking / sync (the pins that ARE connected) ----------
	output logic       vblank,     // active high (-> /SWITCHES D6, IRQ1, snd IRQ,
	output logic       vblank_n,   //               GAL 26R pin 12, watchdog CLK)
	output logic       vsync_n,    // /VSYNC (pin 24) -> 8J F00, 6F 7406

	// ---- frame reset -----------------------------------------------------
	output logic       vres_n,     // /VRES (pin 38) -> VMATCH 17U pin 28 only

	// ---- display coordinates (this core's contract, not a PCB net) -------
	output logic [8:0] vpos,       // 0..239 while `vde`
	output logic       vde,        // vertical display enable = ~vblank

	// ---- markers, one clk_sys cycle wide, aligned to ce_7m ---------------
	output logic       line_start,   // h == 0
	output logic       frame_start,  // h == 0 and v == 0
	output logic       vblank_start  // h == 0 and v == VB_START (VBLANK rising)
);

	// Sized copies of the raster constants, so every compare below is a plain
	// 9-bit unsigned one and no tool has to guess about parameter widths.
	localparam logic [8:0] HTOT = H_TOTAL [8:0];
	localparam logic [8:0] VTOT = V_TOTAL [8:0];
	localparam logic [8:0] VBS  = VB_START[8:0];
	localparam logic [8:0] VSS  = VS_START[8:0];
	localparam logic [8:0] VSE  = VS_END  [8:0];

	// =====================================================================
	// 1. The clock tree inside one count (phase convention: see the header)
	// =====================================================================
	// ph is the position 0..7 inside one count.  `ce_7m` ends a count and
	// `ce_14m` ends a 14M period, so ph is loaded to 0 by ce_7m (the next cycle
	// is the count's first) and to 4 by ce_14m (the next cycle is the second
	// 14M period's first).  Both enables load it, so it re-locks immediately
	// and cannot drift away from the enables it is meant to describe.
	always_ff @(posedge clk) begin
		if      (ce_7m)  ph <= 3'd0;
		else if (ce_14m) ph <= 3'd4;
		else             ph <= ph + 3'd1;
	end

	assign m14   = ~ph[1];
	assign m14_n = ~m14;
	assign m7    = ~ph[2];   // high over ph 0..3 = the first half of the count
	assign m7_n  =  ph[2];

	// ---- MODCLK / MORCLK, solved on this phase ---------------------------
	// 23R half 1: D = 7M, CK = /14MP (= 14M falling, entering ph 2 and ph 6),
	//             /Q = MODCLK.
	//    edge entering ph 2: 7M during ph 1 = 1 -> /Q = 0   (MODCLK low  ph 2..5)
	//    edge entering ph 6: 7M during ph 5 = 0 -> /Q = 1   (MODCLK high ph 6..1)
	// which is exactly "/7M delayed a quarter of a 7M period, 50 % duty".
	assign modclk = (ph >= 3'd6) | (ph <= 3'd1);

	// 18R half 1: D = 7M, CK = 14M (rising entering ph 0 and ph 4),
	//             /PRE = MODCLK (asynchronous, so MORCLK is forced high
	//             whenever MODCLK is low -> ph 2..5), Q = MORCLK.
	//    ph 2..5 : preset      -> 1
	//    edge entering ph 0 : MODCLK high, D = 7M during ph 7 = 0 -> 0
	//    ph 6,7  : no edge, no preset                             -> holds 1
	// => 75 % duty, low for half a 14M period at each 7M rising edge, and
	//    never overlapping MODCLK's low phase, so the line-buffer write and
	//    read strobes are non-overlapping by construction.
	assign morclk = ~(ph <= 3'd1);

	assign ce_m7_rise  = (ph == 3'd7);
	assign ce_m7_fall  = (ph == 3'd3);
	assign ce_m14_rise = (ph == 3'd3) | (ph == 3'd7);
	assign ce_modclk_r = (ph == 3'd5);
	assign ce_morclk_r = (ph == 3'd1);

	// =====================================================================
	// 2. The counters
	// =====================================================================
	// There is no reset on the PCB — the SOS-2 has no reset pin and /VRES is an
	// output — so the counters free-run from power-on.  The synchronous reset
	// below only gives simulation a known starting state.
	wire last_h = (h == HTOT - 9'd1);
	wire last_v = (v  == VTOT - 9'd1);

	always_ff @(posedge clk) begin
		if (reset) begin
			h <= 9'd0;
			v <= 9'd0;
		end else if (ce_7m) begin
			if (last_h) begin
				h <= 9'd0;
				// VCLK = /256H: 256H is high for h = 256..455, so /256H rises
				// at the 455 -> 0 wrap, and advancing V at the line wrap is
				// that rising edge.
				v <= last_v ? 9'd0 : (v + 9'd1);
			end else begin
				h <= h + 9'd1;
			end
		end
	end

	// =====================================================================
	// 3. Delayed horizontal taps — 4HD1H (pin 2) and 4HD2H (pin 19)
	// =====================================================================
	// 4H rises at the 3->4 boundary, so 4HD1H rises at 4->5 and 4HD2H at 5->6.
	// The 4HD2H edge at H = 8c+6 latches the playfield palette in 11V exactly
	// on the count that carries the stamp's pixel 0.
	always_ff @(posedge clk) begin
		if (reset) begin
			h_4hd1h <= 1'b0;
			h_4hd2h <= 1'b0;
		end else if (ce_7m) begin
			h_4hd1h <= h[2];
			h_4hd2h <= h_4hd1h;
		end
	end
	assign h_4hd1h_n = ~h_4hd1h;
	assign h_4hd2h_n = ~h_4hd2h;

	// =====================================================================
	// 4. Vertical blanking, sync and the frame reset
	// =====================================================================
	// Combinational decodes of the V counter.  On the board they come out of
	// the custom already registered; taking them combinationally here keeps
	// every edge exactly on the line boundary (V advances at H = 0).
	wire vb = (v >= VBS);

	assign vblank   =  vb;
	assign vblank_n = ~vb;
	assign vsync_n  = ~((v >= VSS) & (v < VSE));

	// /VRES — the frame reset the SOS-2 shares with the VMATCH custom at 17U,
	// low over the last line of the frame.
	assign vres_n = ~last_v;

	// =====================================================================
	// 5. The individual counter taps
	// =====================================================================
	assign h1     =  h[0];   assign h1_n   = ~h[0];
	assign h2     =  h[1];   assign h2_n   = ~h[1];
	assign h4     =  h[2];   assign h4_n   = ~h[2];
	assign h8     =  h[3];   assign h16    =  h[4];
	assign h32    =  h[5];   assign h64    =  h[6];
	assign h128   =  h[7];   assign h256   =  h[8];
	assign h256_n = ~h[8];                       // -> SOS-2 VCLK, VMATCH

	assign v1     =  v[0];   assign v1_n   = ~v[0];
	assign v2     =  v[1];   assign v4     =  v[2];
	assign v8     =  v[3];   assign v16    =  v[4];
	assign v32    =  v[5];   assign v64    =  v[6];
	assign v128   =  v[7];   assign v256   =  v[8];   // pin 37 is unconnected
	                                                  // on the board; exported
	                                                  // only for completeness

	// =====================================================================
	// 6. Display coordinates and markers
	// =====================================================================
	assign vde  = ~vb;
	assign vpos = v;

	assign line_start   = ce_7m & (h == 9'd0);
	assign frame_start  = line_start & (v == 9'd0);
	assign vblank_start = line_start & (v == VBS);

endmodule
