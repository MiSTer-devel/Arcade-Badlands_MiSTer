`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the MOTION-OBJECT engine: VMATCH 17U, the 13V/15V chip selects
//  and the three stamp ROMs, 17K's palette/priority latch, 18S/2's VMD flop,
//  24J's pen-zero test, GAL 16S + 16P, and the MB87316 line buffer at 15N.
//  Schematic sheet 4.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The 16-count group timeline, n = H mod 16, group g = H[8:4]:
//      n = 0,1    CPU slot.  /8H rising (n = 15 -> 0) -> 17K latches the
//                 MOH palette + priority, 18S/2 latches VMATCHD
//      n = 2,3    PF word
//      n = 4,5    MOP  (the stamp code), complete at n = 5
//      n = 6,7    MOV  (Y + height),     complete at n = 7
//                 PVCLK rising (n = 5 -> 6) -> VMATCH INCLK takes the code,
//                 13V latches code[11:10]
//      n = 8,9    CPU slot.  PVCLK falling (n = 7 -> 8) -> VMATCH INCLK
//                 takes Y + height
//      n = 10,11  PF word
//      n = 12,13  the unused MO bank — fetched and thrown away
//      n = 14,15  MOH (X, priority, palette), complete at n = 15
//                 8HD6H rising (n = 13 -> 14) -> VMATCH OUTCLK issues
//                 RA14:2 + VMATCHD, 15V latches MOA17/16, ROM byte 0
//                 /MOL at n = 15 -> the LB write address <- X
//  The 16 pixels of the object fetched in group g occupy exactly n = 0..15 of
//  group g+1.
//
//  Objects per line — a hard limit with no MAME counterpart:
//      entries 0..26  fetched at H = 0..431, drawn at H = 16..447, fully drawn
//      entry   27     drawn 448..463, truncated by /FIRSTH to 448..453,
//                     so clipped to 6 of its 16 pixels
//      entry   28     MOP and MOV are fetched at H = 452..455 but the MOH slot
//                     (n = 14,15) never occurs, so /MOL never fires and it is
//                     never drawn
//      entries 29-31  never addressed (H[8:4] never reaches them)
//  Nothing here counts objects; the limit is a consequence of the raster.
//
//  The line wrap.  8H, 8HD6H and H mod 16 do not divide 456, so the group
//  beginning at H = 448 is only 8 counts long, and:
//    * /8H produces no rising edge between H = 448 and H = 16, so 17K (palette
//      + priority) and 18S/2 (VMD) hold entry 27's values across the wrap;
//    * 8HD6H does produce a rising edge near the wrap, at H = 0 -> 1: GAL 26R
//      registers its decode and 455 mod 16 = 7, which is not in the decode's
//      set, so 8HD6H is low at both H = 455 and H = 0.  VMATCH therefore
//      re-issues RA14:2 and 15V re-latches MOA17/16 from 13V, which holds
//      entry 28's code[11:10] captured by PVCLK rising at the 453 -> 454
//      boundary;
//    * writes are blocked over H = 454..5 by /FIRSTH, but H = 6..15 is a
//      10-count window in which the line buffer can be written at a stale write
//      address, because no /MOL fired for entry 28.
//  This module uses badlands_hstrobes's edge markers rather than decoding n
//  itself, so it follows the short group by construction.
//============================================================================

module badlands_mo #(
	parameter bit MO_CODE_CARRY = 1'b0,   // see badlands_mo_fetch
	parameter int LB_RD_ADJ     = 0       // see badlands_lb
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_7m,

	// ---- the clock tree ---------------------------------------------------
	input  logic        modclk,        // the MODCLK level, for 16P's /MOW gate
	input  logic        ce_m7_fall,    // LB: publish DO           (ph 3)
	input  logic        ce_modclk_r,   // LB: DCLK rising          (ph 5)
	input  logic        ce_morclk_r,   // LB: RCLK rising          (ph 1)

	// ---- the raster -------------------------------------------------------
	input  logic [3:0]  h,             // 8H, 4H, 2H, 1H
	input  logic        v1,            // 1V  -> RSEL and GAL 16S pin 9
	input  logic        firsth_n,      // GAL 26R pin 17
	input  logic        lasth_n,       // GAL 26R pin 18 (wired to 16S, unused)
	input  logic        line_start,    // VCLK rising, H = 0
	input  logic        vres_n,        // /VRES

	// ---- GAL 26R strobes used as clocks ----------------------------------
	input  logic        pvclk_rise,    // n = 5 -> 6
	input  logic        pvclk_fall,    // n = 7 -> 8
	input  logic        h8d6h,
	input  logic        h8d6h_rise,    // n = 13 -> 14 (and once near the wrap)
	input  logic        h8d6h_fall,

	// ---- VRAM video port -------------------------------------------------
	input  logic [15:0] ramd,

	// ---- gfx client (sprites region) -------------------------------------
	output logic        mo_req,
	output logic [17:0] mo_addr,
	input  logic        mo_ack,
	input  logic [7:0]  mo_data,

	// ---- outputs ---------------------------------------------------------
	output logic [3:0]  mo_pen_n,      // /MO3:0 -> 24J, LB DI3:0, and nothing else
	output logic [7:0]  lb_do,         // DO7:0 -> /CRAMA7:1 and GAL 9N
	output logic        mo_late,

	// ---- observation points ----------------------------------------------
	output logic [8:0]  dline,         // VMATCH's D = the line being prepared
	output logic [12:0] ra_dbg,
	output logic [1:0]  moa_dbg,
	output logic        vmd_dbg,
	output logic [8:0]  lb_wcount,
	output logic [8:0]  lb_rcount,
	output logic        lb_wr_stb,
	output logic [8:0]  lb_wr_addr
);

	// =====================================================================
	// 1. VMATCH 17U
	// =====================================================================
	logic [12:0] ra, ra_next;
	logic [1:0]  chc, chc_next;
	logic        vmatchd_n, vmatchd_n_next;
	// VMATCH publishes its combinational VMATCHD too, but on this board only the
	// OUTCLK-registered pin reaches 18S/2.
	// verilator lint_off UNUSEDSIGNAL
	wire         unused_vmn = vmatchd_n_next;
	// verilator lint_on UNUSEDSIGNAL

	badlands_vmatch u_vmatch (
		.clk(clk), .reset(reset),
		.ramd(ramd),
		.inclk_rise(pvclk_rise), .inclk_fall(pvclk_fall),
		.outclk_rise(h8d6h_rise),
		.line_start(line_start), .vres_n(vres_n),
		.ra(ra), .code_hi_carry(chc), .vmatchd_n(vmatchd_n),
		.ra_next(ra_next), .code_hi_carry_next(chc_next),
		.vmatchd_n_next(vmatchd_n_next),
		.dline(dline));

	assign ra_dbg = ra;

	// =====================================================================
	// 2. The stamp fetch
	// =====================================================================
	badlands_mo_fetch #(.MO_CODE_CARRY(MO_CODE_CARRY)) u_fetch (
		.clk(clk), .reset(reset), .ce_7m(ce_7m),
		.h(h[2:0]),
		.h8d6h(h8d6h), .h8d6h_rise(h8d6h_rise), .h8d6h_fall(h8d6h_fall),
		.pvclk_rise(pvclk_rise),
		.ramd(ramd[11:10]),
		.ra(ra), .ra_next(ra_next),
		.code_hi_carry(chc), .code_hi_carry_next(chc_next),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),
		.mo_pen_n(mo_pen_n), .moa(moa_dbg), .mo_late(mo_late));

	// =====================================================================
	// 3. 17K LS175 and 18S/2 F74 — both clocked by /8H rising
	// =====================================================================
	// /8H rises when 8H falls, i.e. at H mod 16 = 15 -> 0.  8H is already low
	// over H = 448..455, so there is no edge at the line wrap and both
	// registers hold entry 27's values from H = 447 to H = 15.
	wire k17_clk = ce_7m & (h == 4'd15);

	logic [3:0] di_hi;    // 17K /Q3:0 = DI7:DI4 = NOT {RAMD3, RAMD2, RAMD1, RAMD0}
	logic       vmd;      // 18S/2 Q — low means "this entry matches this line"
	always_ff @(posedge clk) begin
		if (reset) begin
			di_hi <= 4'hF;
			vmd   <= 1'b1;
		end else if (k17_clk) begin
			di_hi <= ~ramd[3:0];
			vmd   <= vmatchd_n;
		end
	end
	assign vmd_dbg = vmd;

	// =====================================================================
	// 4. 24J F20 — the pen-zero test (16S pin 11)
	// =====================================================================
	//   MOPENNZ = NAND(/MO3, /MO2, /MO1, /MO0) = 1 when the current pen != 0.
	//   The pen bus is active low, so pen 0 is all ones.
	wire mopennz = ~(&mo_pen_n);

	// =====================================================================
	// 5. GAL 16S + 16P F00
	// =====================================================================
	logic mod0_n, mod1_n, mow0_n, mow1_n, mol0_n, mol1_n, mor0_n, mor1_n;
	logic o14, o15;

	badlands_lb_ctrl u_ctrl (
		.vmd(vmd), .h(h), .lasth_n(lasth_n), .firsth_n(firsth_n), .v1(v1),
		.mopennz(mopennz), .modclk(modclk),
		.mod0_n(mod0_n), .mod1_n(mod1_n),
		.mow0_n(mow0_n), .mow1_n(mow1_n), .o14(o14), .o15(o15),
		.mol0_n(mol0_n), .mor0_n(mor0_n),
		.mol1_n(mol1_n), .mor1_n(mor1_n));

	// The gated /MOW pins are the real nets; the line buffer takes the ungated
	// terms because it latches on the DCLK edge (see badlands_lb_ctrl).
	// verilator lint_off UNUSEDSIGNAL
	wire unused_mow = &{1'b0, mow0_n, mow1_n};
	// verilator lint_on UNUSEDSIGNAL

	// =====================================================================
	// 6. The MB87316 line buffer at 15N
	// =====================================================================
	//   A8:A0 = RAMD15:7 — the MOH word's X field, valid at n = 15 where /MOL
	//   fires.  DI7:DI4 = 17K /Q, DI3:DI0 = /MO3:0.
	badlands_lb #(.RD_ADJ(LB_RD_ADJ)) u_lb (
		.clk(clk), .reset(reset),
		.ce_m7_fall(ce_m7_fall), .ce_modclk_r(ce_modclk_r),
		.ce_morclk_r(ce_morclk_r), .rsel(v1),
		.a(ramd[15:7]), .di({di_hi, mo_pen_n}),
		.mod0_n(mod0_n), .mod1_n(mod1_n),
		.we0(o14), .we1(o15),
		.mol0_n(mol0_n), .mol1_n(mol1_n),
		.mor0_n(mor0_n), .mor1_n(mor1_n),
		.do_out(lb_do), .wcount(lb_wcount), .rcount(lb_rcount),
		.wr_stb(lb_wr_stb), .wr_addr(lb_wr_addr));

endmodule
