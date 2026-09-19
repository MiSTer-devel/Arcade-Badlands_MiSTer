`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the whole raster-time video block: sheets 1 (SOS-2, GAL 26R),
//  3 (playfield), 4 (motion objects, priority, colour RAM) and 6 (the DACs and
//  composite sync).  This is the only module core/rtl/badlands_core.sv has to
//  instantiate for video.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The shared /CRAMA7:1 bus is the one tri-state on the board.  7N LS374 drives
//  it while /PF is low; the MB87316 line buffer's DO6:DO0 drive it while /MO is
//  low.  /MO = NOT /PF out of the same 18R flop, so exactly one source is
//  enabled at every instant and the bus is a plain 2:1 mux here.  DO7 does not
//  reach the bus: it is the stored priority bit and goes only to GAL 9N pin 11.
//
//  The pipeline, end to end, with `ce_7m` marking the last clk_sys cycle of an
//  H count and ph = 0..7 the position inside it:
//
//     count k        7T/7R present /PF3:0 and /MO3:0 for count k;
//                    the LB presents DO for count k from ph 4
//     ph 4..7 of k   GAL 9N evaluates the rule (7M = 0) on those pens
//     ph 0..3 of k+1 18R publishes the winner; 7N (latched at ph 4 of k) or the
//                    LB drives /CRAMA7:1
//     ph 4 of k+1    10K latches {PRIO, /CRAMA7:1}; 8K reads byte 0 over
//                    ph 4..7 and byte 1 over ph 0..3 of k+2
//     ph 4 of k+2    PIXCLK loads 1K/5K -> the DACs
//     ce_7m of k+2   this module registers the DAC output
//
//  So the pen presented on count k is displayed at visible pixel k - 6.  Through
//  both layers that means:
//    * a playfield stamp fetched in group c puts its pixels on /PF3:0 at
//      H = 8c+6 .. 8c+13, so tile column c occupies screen pixels 8c .. 8c+7 —
//      the visible window is columns 0..41 exactly, with no scroll and no
//      offset, which is what MAME's un-scrolled 64 x 32 TILEMAP_SCAN_ROWS
//      produces;
//    * the LB read pointer is reset at H = 5 and location L is read during
//      count L+6, so LB location L is displayed at screen pixel L — the MOH
//      word's X field maps straight to the screen column, as MAME's atarimo
//      does with no xoffset.
//
//  Blanking and sync.  The board has one blanking net, /BLANK (GAL 26R pin 13),
//  and it already contains VBLANK.  It clears 1K/5K, which is the only blanking
//  mechanism — sheet 6 has no /BLANK input of its own.  MiSTer's `arcade_video`
//  wants separate hblank and vblank, so this module exports GAL 26R's own sum
//  with the VBLANK term removed as `hblank`, and the SOS-2's VBLANK as
//  `vblank`.  Composite sync is /SYNC = /HSYNC AND /VSYNC (8J F00 NAND + 6F
//  7406 inverter).
//============================================================================

module badlands_video #(
	parameter int  V_TOTAL       = 262,     // lines per frame
	parameter int  HSYNC_TAP     = 1,       // 13V delay tap; R98 is the fitted part
	parameter bit  BANK_POR      = 1'b0,    // power-on value of the 18S bank flop
	parameter bit  MO_CODE_CARRY = 1'b0,    // 0 = schematic, 1 = MAME's row carry
	parameter int  LB_RD_ADJ     = 0,       // line-buffer-to-H alignment
	parameter bit  DAC_VTAB      = 1'b0     // 1 = the analogue DAC voltage table
)(
	input  logic        clk,                // clk_sys 57.272724 MHz
	input  logic        reset,
	input  logic        ce_14m,             // clk_sys/4
	input  logic        ce_7m,              // clk_sys/8 — one pixel, one H count

	// ---- VRAM video port -------------------------------------------------
	input  logic [15:0] ramd,
	input  logic        ramd_pf_stb,        // h mod 8 == 3: the PF word
	input  logic        ramd_mo0_stb,       // h mod 8 == 5: MO bank {8H,0}
	input  logic        ramd_mo1_stb,       // h mod 8 == 7: MO bank {8H,1}

	// ---- gfx clients -----------------------------------------------------
	output logic        pf_req,
	output logic [18:0] pf_addr,
	input  logic        pf_ack,
	input  logic [7:0]  pf_data,
	output logic        mo_req,
	output logic [17:0] mo_addr,
	input  logic        mo_ack,
	input  logic [7:0]  mo_data,

	// ---- colour RAM CPU port ---------------------------------------------
	input  logic        cram_cs,
	input  logic [9:1]  cram_a,
	input  logic        cram_we,
	input  logic [7:0]  cram_din,
	output logic [7:0]  cram_dout,

	// ---- 18S playfield bank flop (0xFEC001 write, D0) --------------------
	input  logic        pfbank_wr,
	input  logic        pfbank_d0,

	// ---- GAL 9N pin 12: AUDFULL on D8 at 0xFC0000 ------------------------
	input  logic        misc2_n,
	input  logic        rw,
	input  logic        audfull,
	output logic        audfull_d8,
	output logic        audfull_d8_oe,

	// ---- video out, all registered together at ce_7m ---------------------
	output logic        hblank,
	output logic        vblank,
	output logic        hsync_n,
	output logic        vsync_n,
	output logic        csync_n,            // /SYNC to JAMMA-P / AVID-9
	output logic [7:0]  red,
	output logic [7:0]  green,
	output logic [7:0]  blue,
	output logic [7:0]  pix_index,       // not a PCB net: the colour index that
	                                     // produced this pixel, simulation only

	// ---- raster taps the rest of the board needs -------------------------
	output logic [8:0]  h,                  // also feeds badlands_gfx_mem's
	output logic [8:0]  v,                  //   h_mod8
	output logic        v32,                // -> the 6502 IRQ
	output logic        vblank_raw,         // the SOS-2 pin, unregistered:
	                                        //   /SWITCHES D6, the watchdog clock
	output logic        virq_edge,          // VBLANK rising -> 68000 IRQ1
	output logic        gfx_late            // either client missed its window
);

	// =====================================================================
	// 1. The SOS-2 (25M) and the 23R/18R clock flops
	// =====================================================================
	logic m7;
	logic modclk;
	logic ce_m7_rise, ce_m7_fall, ce_m14_rise, ce_modclk_r, ce_morclk_r;
	logic v1;
	logic vres_n, line_start, frame_start, vblank_start;
	logic vblank_i;

	/* verilator lint_off UNUSEDSIGNAL */
	logic m14, m14_n, m7_n, morclk, v1_n;
	logic [2:0] ph;
	logic h1, h1_n, h2, h2_n, h4, h4_n, h8, h16, h32, h64, h128, h256, h256_n;
	logic v2, v4, v8, v16, v64, v128, v256;
	logic h_4hd1h, h_4hd1h_n, h_4hd2h, h_4hd2h_n;
	logic vblank_n, vde;
	logic [8:0] vpos;
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_sos2_sync #(.V_TOTAL(V_TOTAL)) u_sync (
		.clk(clk), .reset(reset), .ce_14m(ce_14m), .ce_7m(ce_7m),
		.m14(m14), .m14_n(m14_n), .m7(m7), .m7_n(m7_n), .ph(ph),
		.modclk(modclk), .morclk(morclk),
		.ce_m7_rise(ce_m7_rise), .ce_m7_fall(ce_m7_fall),
		.ce_m14_rise(ce_m14_rise),
		.ce_modclk_r(ce_modclk_r), .ce_morclk_r(ce_morclk_r),
		.h(h), .v(v),
		.h1(h1), .h1_n(h1_n), .h2(h2), .h2_n(h2_n), .h4(h4), .h4_n(h4_n),
		.h8(h8), .h16(h16), .h32(h32), .h64(h64), .h128(h128), .h256(h256),
		.h256_n(h256_n),
		.v1(v1), .v1_n(v1_n), .v2(v2), .v4(v4), .v8(v8), .v16(v16),
		.v32(v32), .v64(v64), .v128(v128), .v256(v256),
		.h_4hd1h(h_4hd1h), .h_4hd1h_n(h_4hd1h_n),
		.h_4hd2h(h_4hd2h), .h_4hd2h_n(h_4hd2h_n),
		.vblank(vblank_i), .vblank_n(vblank_n), .vsync_n(vsync_raw),
		.vres_n(vres_n), .vpos(vpos), .vde(vde),
		.line_start(line_start), .frame_start(frame_start),
		.vblank_start(vblank_start));

	logic vsync_raw;
	assign vblank_raw = vblank_i;
	assign virq_edge  = vblank_start;      // VBLANK rising, one clk_sys cycle

	// =====================================================================
	// 2. GAL 26R (26R) + the 13J inverter + the 13V /HSYNC delay
	// =====================================================================
	logic blank_n, hsyncdo_n, h8d6h_n, pvclk, firsth_n, lasth_n, h8d6h;
	logic hsync_raw, hblank_raw;
	logic pvclk_rise, pvclk_fall, h8d6h_rise, h8d6h_fall;

	/* verilator lint_off UNUSEDSIGNAL */
	logic unused_strobes;
	assign unused_strobes = &{1'b0, hsyncdo_n, h8d6h_n, pvclk};
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_hstrobes #(.HSYNC_TAP(HSYNC_TAP)) u_hs (
		.clk(clk), .reset(reset), .ce_7m(ce_7m),
		.h(h), .vblank(vblank_i),
		.blank_n(blank_n), .hsyncdo_n(hsyncdo_n), .h8d6h_n(h8d6h_n),
		.pvclk(pvclk), .firsth_n(firsth_n), .lasth_n(lasth_n),
		.h8d6h(h8d6h), .hsync_n(hsync_raw), .hblank(hblank_raw),
		.pvclk_rise(pvclk_rise), .pvclk_fall(pvclk_fall),
		.h8d6h_rise(h8d6h_rise), .h8d6h_fall(h8d6h_fall));

	// =====================================================================
	// 3. The playfield (sheet 3)
	// =====================================================================
	logic [3:0] pf_pen_n;
	logic [2:0] pf_pal_n;
	logic [6:0] cram_pf_n;
	logic       pf_bank, pf_late;

	/* verilator lint_off UNUSEDSIGNAL */
	logic unused_pf;
	assign unused_pf = &{1'b0, pf_pal_n, pf_bank, pf_pen_n[2:0]};
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_pf #(.BANK_POR(BANK_POR)) u_pf (
		.clk(clk), .reset(reset), .ce_7m(ce_7m), .ce_m7_fall(ce_m7_fall),
		.h(h[2:0]), .v(v[2:0]),
		.ramd(ramd), .ramd_pf_stb(ramd_pf_stb),
		.pfbank_wr(pfbank_wr), .pfbank_d0(pfbank_d0),
		.pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack), .pf_data(pf_data),
		.pf_pen_n(pf_pen_n), .pf_pal_n(pf_pal_n), .cram_pf_n(cram_pf_n),
		.pf_bank(pf_bank), .pf_late(pf_late));

	// =====================================================================
	// 4. The motion objects (sheet 4)
	// =====================================================================
	logic [3:0] mo_pen_n;
	logic [7:0] lb_do;
	logic       mo_late;

	/* verilator lint_off UNUSEDSIGNAL */
	logic [8:0]  mo_dline, mo_wcount, mo_rcount, mo_wr_addr;
	logic [12:0] mo_ra;
	logic [1:0]  mo_moa;
	logic        mo_vmd, mo_wr_stb;
	logic        unused_mo;
	assign unused_mo = &{1'b0, mo_dline, mo_wcount, mo_rcount, mo_wr_addr,
	                     mo_ra, mo_moa, mo_vmd, mo_wr_stb,
	                     ramd_mo0_stb, ramd_mo1_stb, mo_pen_n};
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_mo #(.MO_CODE_CARRY(MO_CODE_CARRY), .LB_RD_ADJ(LB_RD_ADJ)) u_mo (
		.clk(clk), .reset(reset), .ce_7m(ce_7m),
		.modclk(modclk), .ce_m7_fall(ce_m7_fall),
		.ce_modclk_r(ce_modclk_r), .ce_morclk_r(ce_morclk_r),
		.h(h[3:0]), .v1(v1), .firsth_n(firsth_n), .lasth_n(lasth_n),
		.line_start(line_start), .vres_n(vres_n),
		.pvclk_rise(pvclk_rise), .pvclk_fall(pvclk_fall),
		.h8d6h(h8d6h), .h8d6h_rise(h8d6h_rise), .h8d6h_fall(h8d6h_fall),
		.ramd(ramd),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),
		.mo_pen_n(mo_pen_n), .lb_do(lb_do), .mo_late(mo_late),
		.dline(mo_dline), .ra_dbg(mo_ra), .moa_dbg(mo_moa), .vmd_dbg(mo_vmd),
		.lb_wcount(mo_wcount), .lb_rcount(mo_rcount),
		.lb_wr_stb(mo_wr_stb), .lb_wr_addr(mo_wr_addr));

	assign gfx_late = pf_late | mo_late;

	// =====================================================================
	// 5. The shared /CRAMA7:1 bus and GAL 9N (sheet 4)
	// =====================================================================
	logic pf_n, mo_n, prio, pin19;

	/* verilator lint_off UNUSEDSIGNAL */
	logic unused_prio;
	assign unused_prio = &{1'b0, mo_n, pin19};
	/* verilator lint_on UNUSEDSIGNAL */

	// 7N drives while /PF is low; the LB's DO6:DO0 drive while /MO is low.
	wire [6:0] crama_n = pf_n ? lb_do[6:0] : cram_pf_n;

	badlands_prio u_prio (
		.clk(clk), .reset(reset),
		.m7(m7), .crama_n(crama_n), .pf3_n(pf_pen_n[3]), .do7(lb_do[7]),
		.misc2_n(misc2_n), .rw(rw), .audfull(audfull),
		.d8(audfull_d8), .d8_oe(audfull_d8_oe),
		.ce_m14_rise(ce_m14_rise),
		.pf_n(pf_n), .mo_n(mo_n), .prio(prio), .pin19(pin19));

	// =====================================================================
	// 6. Colour RAM and the DACs (sheets 4 and 6)
	// =====================================================================
	logic [7:0] cbyte0, cbyte1, cram_index;
	logic       pixclk_rise;

	/* verilator lint_off UNUSEDSIGNAL */
	logic unused_pix;
	assign unused_pix = &{1'b0, pixclk_rise};
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_cram u_cram (
		.clk(clk), .reset(reset),
		.m7(m7), .ce_m7_rise(ce_m7_rise), .ce_m7_fall(ce_m7_fall),
		.blank_n(blank_n),
		.crama_n(crama_n), .prio(prio),
		.cram_cs(cram_cs), .cram_a(cram_a), .cram_we(cram_we),
		.cram_din(cram_din), .cram_dout(cram_dout),
		.byte0(cbyte0), .byte1(cbyte1), .pixclk_rise(pixclk_rise),
		.pix_index(cram_index));

	logic [7:0] dac_r, dac_g, dac_b;
	/* verilator lint_off UNUSEDSIGNAL */
	logic [4:0] dac_r5, dac_b5;
	logic [5:0] dac_g6;
	logic       unused_dac;
	assign unused_dac = &{1'b0, dac_r5, dac_g6, dac_b5};
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_dac #(.VOLTAGE_TABLE(DAC_VTAB)) u_dac (
		.byte0(cbyte0), .byte1(cbyte1),
		.red(dac_r), .green(dac_g), .blue(dac_b),
		.red5(dac_r5), .green6(dac_g6), .blue5(dac_b5));

	// =====================================================================
	// 7. The output registers — everything sampled on the same ce_7m tick
	// =====================================================================
	// The DACs settle at PIXCLK (ph 4) and the blanking / sync nets are per-count
	// values out of GAL 26R, so one common ce_7m register keeps them aligned to
	// the count and hands `arcade_video` a clean one-pixel-per-ce_pix stream.
	logic [7:0] pat_r, pat_g, pat_b;
`ifdef BADLANDS_TESTPAT
	// The bring-up test pattern, never compiled into a build unless
	// BADLANDS_TESTPAT is defined.
	badlands_testpat u_testpat (
		.clk(clk), .reset(reset), .ce(ce_7m),
		.h_count(h), .v_count(v),
		.hblank(hblank_raw), .vblank(vblank_i), .frame_start(frame_start),
		.r(pat_r), .g(pat_g), .b(pat_b));
`else
	assign pat_r = 8'h00;
	assign pat_g = 8'h00;
	assign pat_b = 8'h00;
	/* verilator lint_off UNUSEDSIGNAL */
	logic unused_fs;
	assign unused_fs = &{1'b0, frame_start, pat_r, pat_g, pat_b};
	/* verilator lint_on UNUSEDSIGNAL */
`endif

	always_ff @(posedge clk) begin
		if (reset) begin
			hblank  <= 1'b1;
			vblank  <= 1'b1;
			hsync_n <= 1'b1;
			vsync_n <= 1'b1;
			csync_n <= 1'b1;
			red     <= 8'h00;
			green   <= 8'h00;
			blue    <= 8'h00;
			pix_index <= 8'h00;
		end else if (ce_7m) begin
			pix_index <= cram_index;
			hblank  <= hblank_raw;
			vblank  <= vblank_i;
			hsync_n <= hsync_raw;
			vsync_n <= vsync_raw;
			// 8J F00 NAND(/HSYNC, /VSYNC) -> 6F 7406 inverter
			csync_n <= hsync_raw & vsync_raw;
`ifdef BADLANDS_TESTPAT
			red     <= pat_r;
			green   <= pat_g;
			blue    <= pat_b;
`else
			red     <= dac_r;
			green   <= dac_g;
			blue    <= dac_b;
`endif
		end
	end

endmodule
