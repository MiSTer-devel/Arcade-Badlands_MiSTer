// Bad Lands analog-output alignment stage: the OSD decode, the read-rate
// generator and the blanking glue around rmonic79's `crt_adjust`
// (rtl/video/crt_adjust.sv, vendored from MiSTer-CRT-Adjust c682de9).  Written
// from that project's examples/core_side_snippet.v (GPL-3.0, Umberto Parisi)
// by way of the Blasteroids core's blstroid_analog_adjust.sv, re-derived for an
// 8-clock pixel and for feeding arcade_video rather than the VGA_* pins.
//
// Copyright (C) 2026 RetroShrimp.  GPL-3.0-or-later; see LICENSE and NOTICE.md.
//
// The OSD carries three controls plus an On/Off: CRT H-Size, CRT H-Position and
// CRT V-Shift.  H-Position has two possible mechanisms inside `crt_adjust`,
// picked at compile time by `HPOS_MODE`:
//
//   HPOS_CONTENTSHIFT (1)  shifts the CONTENT in the line buffer, HSync native
//   HPOS_SYNCSHIFT    (0)  shifts the HSYNC, content native
//
// Only one of the two may be built, and the whole engine restarts on one line
// reference, `hs_ref_out`.  The read-rate accumulator below must be reset on
// that edge rather than on the raw HSync, or the picture trembles.
//
// None of it touches the core's own timing: the pixel clock and the native sync
// rates are untouched, so there is no jitter and nothing the game can see.
//
// ---------------- HPOS_MODE for Bad Lands: CONTENTSHIFT ---------------------
// Upstream's rule is "CONTENTSHIFT for wide/centred games, SYNCSHIFT for narrow
// side-anchored ones, because CONTENTSHIFT runs the picture out of the buffer
// window on the short-margin side and shows a black block there".  Our numbers
// (badlands_hstrobes, GAL 26R + 13V; one pixel per H count):
//
//     line total        456 px
//     active            H 8 .. 343            (336 px, /BLANK high)
//     HSync low         H 374 .. 405          (/HSYNC, after 13V)
//
// The line buffer's write pointer resets on the HSync rise, so in BUFFER
// coordinates the active region is 90 .. 425 of 456: a 90-pixel lead-in and a
// 30-pixel run-out.  The short margin is the RIGHT one, and it is short:
//
//   * at H-Size 0 a line delivers exactly 456 read ticks, so a positive
//     H-Position beyond +30 runs the right-hand columns off the end of the line
//     (+63 loses 33 of them);
//   * enlarging slows the read, 456 * 64 / (64 + hsize) ticks a line, so at
//     H-Position 0 everything up to H-Size +4 fits (429 ticks) and +5 starts to
//     clip; +15 (369 ticks) needs an H-Position of about -56 to bring the right
//     edge back.  The 90-pixel lead-in is what pays for that.
//
// That is the same limit the retired analog_hsize stage had (it also shifted
// the content).  CONTENTSHIFT stays the default: it is upstream's recommended
// mode, it is what the Blasteroids core builds, and only it can buy back room
// for an enlarged picture -- SYNCSHIFT moves the sync and the content together,
// so it re-centres the picture on the tube but never changes what fits in a
// line.  If a real CRT ever shows a black block at the right edge, flip
// HPOS_MODE to 0 here and nothing else changes.
//
// ---------------- the read rate, and why 64/CLK_PER_PIX ---------------------
// Upstream steps the read period in quarter clocks because its reference core's
// pixel is 16 clk_sys cycles: 64 quarters = one pixel, so one H-Size click is
// 1/64 = 1.56%.  Ours is 8 (ce_7m = clk_sys/8 at 57.27 MHz), and a fixed
// quarter-clock step would make one click 3.1% instead.  The accumulator below
// counts 1/CLK_PER_PIX-th of a clock instead, which keeps the 1.56% step at any
// pixel period:
//
//     period = 64 + hsize      units, where 64 units == CLK_PER_PIX clocks
//     acc   += 64/CLK_PER_PIX  per clock
//
// so hsize spans 48..79 units = 6.00..9.88 clocks per pixel at CLK_PER_PIX = 8.
// ACC_STEP must stay an exact integer -- 64/CLK_PER_PIX with CLK_PER_PIX a
// power of two -- or the OSD step silently drifts away from the menu text.
//
// ---------------- the scandoubler ----------------
// While On, the engine's hb_out is `~pass_q`, which is low only inside the
// active window of an ACTIVE line: VBLANK is folded back into HBLANK, and the
// pixel enable is the uneven 8/9-clock read tick.  The MiSTer scandoubler
// cannot take either (see the hb_in note below), so Arcade-Badlands.sv turns
// the stage Off whenever any Scandoubler Fx or the forced scandoubler is in
// use.  It is a 15 kHz adjustment anyway.
//
// All three OSD fields are plain two's complement, so the config string just
// lists 0,+1..+max then -min..-1 and the decode is a $signed() -- no bespoke
// mapping to get out of step with the menu text.

module badlands_analog_adjust #(
	parameter int H_TOTAL     = 456,   // ce_pix ticks per line (badlands_sos2_sync: 456)
	parameter int V_TOTAL     = 262,   // lines per frame       (badlands_sos2_sync: 262)
	parameter int CLK_PER_PIX = 8,     // clk_sys cycles per ce_pix (ce_7m = clk_sys/8)
	parameter int HPOS_MODE   = 1,     // 1 = CONTENTSHIFT, 0 = SYNCSHIFT (see above)
	parameter int AW          = 11     // line-buffer address bits (1024 per bank)
) (
	input  logic       clk,
	input  logic       ce_pix,

	// OSD
	input  logic       osd_on,         // CRT Adjust Off/On; Off is a bit-exact bypass
	input  logic [4:0] osd_hsize,      // CRT H-Size        +15 / -16   two's complement
	input  logic [6:0] osd_hpos,       // CRT H-Position    +63 / -64   two's complement
	input  logic [5:0] osd_vshift,     // CRT V-Shift       +31 / -32   two's complement

	input  logic [7:0] r_in, g_in, b_in,
	input  logic       hs_in, vs_in, hb_in, vb_in,

	output logic [7:0] r_out, g_out, b_out,
	output logic       hs_out, vs_out, hb_out, vb_out,
	output logic       ce_out
);

	localparam [7:0] ACC_STEP = 8'(64 / CLK_PER_PIX);

	// ---------------- OSD decode ----------------
	// Everything is sampled at ce_pix so a mid-frame menu change lands on a
	// pixel boundary rather than anywhere inside one.
	logic              active;
	logic signed [4:0] hsize_s;
	logic signed [8:0] hpos_s;
	logic signed [5:0] vshift_s;

	always_ff @(posedge clk) if (ce_pix) begin
		active   <= osd_on;
		hsize_s  <= $signed(osd_hsize);
		hpos_s   <= 9'($signed(osd_hpos));    // sign-extend 7 -> 9 bits
		vshift_s <= $signed(osd_vshift);
	end

	// ---------------- read-rate generator ----------------
	// The wiring rule: reset the accumulator on the rise of the module's
	// `hs_ref_out`, never on the raw HSync, so the write side, the module's read
	// counter and this read rate all restart on the same edge.
	wire  hs_ref;
	logic hs_ref_d;
	always_ff @(posedge clk) hs_ref_d <= hs_ref;
	wire  hs_ref_rise = hs_ref & ~hs_ref_d;

	wire  [7:0] rd_period = 8'd64 + {{3{hsize_s[4]}}, hsize_s};   // 48..79
	logic [7:0] rd_acc;
	wire        rd_tick = (rd_acc + ACC_STEP) >= rd_period;

	always_ff @(posedge clk) begin
		if      (hs_ref_rise) rd_acc <= 8'd0;
		else if (rd_tick)     rd_acc <= rd_acc + ACC_STEP - rd_period;
		else                  rd_acc <= rd_acc + ACC_STEP;
	end

	// When On the line buffer is always live -- even at H-Size 0, where rd_tick
	// runs at the same average rate as ce_pix but re-phased onto hs_ref.
	wire rd_ce = active ? rd_tick : ce_pix;

	// ---------------- the engine ----------------
	wire [7:0] str_r, str_g, str_b;
	wire       str_hs, str_vs, str_hb, str_vb;

	crt_adjust #(
		.VTOTAL    (V_TOTAL),
		.HTOTAL    (H_TOTAL),
		.HPOS_MODE (HPOS_MODE),
		.AW        (AW),
		.VB_LATE   (0)              // HSync follows the active pixels here
	) u_crt_adjust (
		.clk        (clk),
		.pxl_cen    (ce_pix),
		.pxl2_cen   (rd_ce),
		.active     (active),
		.hsize      (hsize_s),      // kept for upstream parity; the rate above is H-Size
		.hoffset    (hpos_s),
		.voffset    (vshift_s),
		.r_in       (r_in), .g_in (g_in), .b_in (b_in),
		.hs_in      (hs_in),        // NATIVE sync in -- the module derives hs_ref itself
		.vs_in      (vs_in),
		// Plain HBlank, not hb_in | vb_in.  Off, this is what reaches
		// arcade_video, and the MiSTer scandoubler needs HBlank and VBlank
		// separate: arcade_video samples VBlank only on an HBlank falling edge,
		// and scandoubler.v measures hs_end from a counter reset only on that
		// edge.  With HBlank held through vertical blank, hs_end runs away
		// during VBLANK, the VSync pulse is never shifted out and the doubled
		// output has no vertical sync at all -- every Scandoubler Fx setting but
		// None comes out corrupt (tests/tb_badlands_scandoubler guards it).  On,
		// the engine only takes the active window's edges from it, which are the
		// same H positions on every line, and gates VBLANK itself from vb_in.
		.hb_in      (hb_in),
		.vb_in      (vb_in),        // TRUE vertical blank, not the combined blank
		.r_out      (str_r), .g_out (str_g), .b_out (str_b),
		.hs_out     (str_hs), .vs_out (str_vs),
		.hb_out     (str_hb), .vb_out (str_vb),
		.hs_ref_out (hs_ref)
	);

	// ---------------- the vertical blank handed to arcade_video ------------
	// On this raster the V counter and VBLANK advance at the 455 -> 0 wrap and
	// HSync comes AFTER the active pixels, so the bank that closes at line L's
	// HSync rise already holds line L and is emitted during native line L+1.
	// crt_adjust is built with VB_LATE = 0 for exactly that: its read-side gate
	// samples vb_in once, on that HSync rise.  `str_vb` is not that signal -- it
	// is the native, write-rate blank, one line early for the buffered picture
	// -- so handing it to arcade_video (which latches VBlank where HBlank falls)
	// would lose the bottom line.  `vb_read` is a bit-exact replica of the
	// engine's own gate, sampled on the same `hs_ref` edge, so DE goes low over
	// exactly the lines `pass_q` is already suppressing.  It must stay ONE flop
	// while VB_LATE = 0; a second flop shows a stale bank below the picture.
	logic hs_ref_q;
	always_ff @(posedge clk) if (ce_pix) hs_ref_q <= hs_ref;
	wire  ref_line_tick = ce_pix & hs_ref & ~hs_ref_q;

	logic vb_read;
	always_ff @(posedge clk) if (ref_line_tick) vb_read <= vb_in;   // one flop, = VB_LATE 0

	// ---------------- what is NOT here: upstream's `de_osd` ----------------
	// Upstream's core-side snippet hands the framework a DE window whose rising
	// edge is anchored to the native active region and whose falling edge
	// follows the stretched one, so the MiSTer OSD -- which centres itself on
	// the rise of VGA_DE -- stays put while H-Position moves the picture.  It is
	// a real improvement and it does not work on this raster.
	//
	// It assumes the stretched active never STARTS before the native one.  Our
	// active region sits 90 pixels into the line buffer (see the HPOS_MODE
	// discussion above), and pulling the content left is exactly what a negative
	// H-Position is for here -- it is the direction that buys back room when
	// H-Size enlarges.  The moment `hoffset` goes negative the content opens
	// before the window does, and everything to the left of the native active
	// start is DE-blanked: the picture loses its left edge, and if the content
	// finishes before the native rise the window opens on nothing and stays open
	// across HSync.  (Measured on the Blasteroids raster, which has the same
	// long lead-in shape.)
	//
	// So DE here simply follows the picture.  The cost is the one upstream set
	// out to remove: the OSD slides with H-Position.  Anyone re-attempting the
	// anchored window has to make its rising edge the EARLIER of the native and
	// the stretched active, and suppress the native rise on a line whose
	// stretched active has already ended -- do not just copy the snippet back in.

	// ---------------- outputs ----------------
	// Off: `crt_adjust` is in its own pure bypass, so everything here is the
	// native stream delayed by the module's two registers.
	assign r_out  = str_r;
	assign g_out  = str_g;
	assign b_out  = str_b;
	assign hs_out = str_hs;
	assign vs_out = str_vs;
	assign hb_out = str_hb;
	assign vb_out = active ? vb_read : str_vb;
	assign ce_out = rd_ce;

endmodule
