`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the motion-object LINE BUFFER, Fujitsu MB87316 at 15N.
//  Schematic sheet 4 grid 15N.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  Pins:
//      A8:A0 = RAMD15:RAMD7 (the MOH word's X field)   ACLK = 7M
//      RCLK  = MORCLK      DCLK = MODCLK               RSEL = 1V
//      WE1/WE0 = /MOW1 //MOW0     DIE1/DIE0 = /MOD1 //MOD0
//      MOR1/MOL1 = /MOR1 //MOL1   MOR0/MOL0 = /MOR0 //MOL0
//      DI7:DI4 = 17K /Q = NOT {RAMD3,RAMD2,RAMD1,RAMD0} = /priority, /palette
//      DI3:DI0 = /MO3:0  (7T/7R high nibble)
//      DOE = /MO   (the tri-state itself lives in badlands_video.sv)
//      DO7 -> GAL 9N pin 11 (the stored priority bit)
//      DO6:DO4 -> /CRAMA7:5      DO3:DO0 -> /CRAMA4:1 and GAL 9N pins 5,4,3,2
//
//  512 locations per bank (A8:A0 = 9 bits) = MAME's `xposmask = 0xff80`.
//
//  Behaviour:
//      write side (bank w = the one NOT selected by RSEL):
//          on /MOL_w  : wcount_w <- A8:0             (the object's X)
//          on each DCLK: if /MOW_w: bank_w[wcount_w] <- DI ; wcount_w++
//          DI passes through only while DIE_w is asserted; with DIE_w
//          deasserted the data path presents its idle value.
//      read side (bank r = RSEL = 1V):
//          on /MOR_r  : rcount_r <- 0
//          on each read: DO <- bank_r[rcount_r] ; rcount_r++
//          the read bank's WE is asserted unconditionally with DIE deasserted,
//          so the location just read is overwritten with the idle value:
//          clear-on-read.  There is no other pass in which the display bank
//          could be cleared, and the display must start each line blank.
//
//  The idle value is 8'hFF: DI3:0 are the active-low pens /MO3:0, so all-ones
//  decodes to pen 0 = transparent, matching MAME's transparent pen 0.  Any
//  other value would make a cleared buffer opaque.
//
//  Order within a count: MODCLK is high over ph 6,7,0,1 and MORCLK low over
//  ph 0,1 only, so the two strobes never overlap.  One write and one read
//  happen per H count.  The write is taken on the MODCLK rising edge (ph 6),
//  using that count's pen and that count's /MOW, and the /MOL load is applied
//  AFTER the increment — which is what makes an object whose X is loaded at
//  n = 15 occupy exactly locations X .. X+15 over n = 0..15 of the next group.
//
//  Read phase, the one degree of freedom this module exposes.  MORCLK (RCLK)
//  rising, at ph 2 of every count, reads `bank_r[rcount]` and advances the
//  pointer; ACLK (= 7M) is the part's own reference clock and its falling edge,
//  at ph 4, publishes that value on DO.  The output register has to sit on the
//  7M falling edge, because that is the only phase at which DO is stable across
//  the whole window
//
//      [7M falling of count k  ..  7M falling of count k+1]
//
//  in which GAL 9N evaluates the priority (during the 7M = 0 half of count k)
//  and 10K latches the winner's index (at the next 7M falling edge).  Updating
//  DO on either MORCLK edge instead would hand the colour RAM an MO index one
//  pixel away from the one the priority logic tested, and would also misalign
//  it against 7N, which latches the playfield index on that very same 7M
//  falling edge.  With this phase the MOH X field maps straight to the screen
//  column with no offset, exactly as MAME's atarimo does.  `RD_ADJ` sweeps
//  that alignment by +-1.
//============================================================================

module badlands_lb #(
	// The LB-location-to-H alignment.  0 = the derived value (see the header);
	// +-1 are the neighbouring candidates.
	parameter int RD_ADJ = 0,
	// The data-in idle value.  8'hFF is forced by the transparency requirement.
	parameter logic [7:0] IDLE = 8'hFF
)(
	input  logic       clk,
	input  logic       reset,

	// The three clock edges, as one-cycle enables from badlands_sos2_sync.  Each
	// enable is high in the clk_sys cycle before its edge (ph 1, 3, 5), so the
	// registered action lands on the edge itself: read (ph 2) -> publish (ph 4)
	// -> write (ph 6), one of each per H count.
	input  logic       ce_m7_fall,  // ACLK = 7M falling: publish DO
	input  logic       ce_modclk_r, // DCLK rising
	input  logic       ce_morclk_r, // RCLK rising
	input  logic       rsel,        // RSEL = 1V

	input  logic [8:0] a,           // A8:A0 = RAMD15:7
	input  logic [7:0] di,          // DI7:0

	input  logic       mod0_n, mod1_n,   // DIE0 / DIE1
	input  logic       we0, we1,         // 16S o14 / o15: /MOW before the
	                                     // MODCLK gate (see badlands_lb_ctrl)
	input  logic       mol0_n, mol1_n,   // MOL0 / MOL1
	input  logic       mor0_n, mor1_n,   // MOR0 / MOR1

	output logic [7:0] do_out,      // DO7:0
	output logic [8:0] wcount,      // the active write pointer (observation)
	output logic [8:0] rcount,      // the active read pointer  (observation)
	output logic       wr_stb,      // a write to the write bank happened
	output logic [8:0] wr_addr      // ...at this location
);

	// Two independent 512 x 8 banks.
	logic [7:0] mem0 [0:511];
	logic [7:0] mem1 [0:511];

	logic [8:0] wcnt0, wcnt1;
	logic [8:0] rcnt0, rcnt1;

	// bank r = RSEL (the display bank), bank w = the other one
	wire rb = rsel;

	// per-bank strobes, resolved to the active bank of each side
	wire w_mol_n = rb ? mol0_n : mol1_n;   // the write bank is ~RSEL
	wire w_we    = rb ? we0    : we1;
	wire w_die_n = rb ? mod0_n : mod1_n;
	wire r_mor_n = rb ? mor1_n : mor0_n;   // the read bank is RSEL
	wire r_we    = rb ? we1    : we0;
	wire r_die_n = rb ? mod1_n : mod0_n;

	wire [7:0] w_data = w_die_n ? IDLE : di;   // DIE deasserted -> the idle value
	wire [7:0] r_data = r_die_n ? IDLE : di;

	// =====================================================================
	// Write side — one write per H count, on the DCLK rising edge
	// =====================================================================
	// The /MOL load is applied AFTER the increment, which is what makes an
	// object whose X is loaded at n = 15 occupy exactly X .. X+15.
	wire [8:0] wc0_nxt = w_mol_n ? (wcnt0 + 9'd1) : a;
	wire [8:0] wc1_nxt = w_mol_n ? (wcnt1 + 9'd1) : a;
	always_ff @(posedge clk) begin
		if (reset) begin
			wcnt0 <= 9'd0;
			wcnt1 <= 9'd0;
		end else if (ce_modclk_r) begin
			if (rb) wcnt0 <= wc0_nxt;           // RSEL = 1 -> bank 0 is written
			else    wcnt1 <= wc1_nxt;
		end
	end

	// =====================================================================
	// The two arrays: one write port and one read port each
	// =====================================================================
	// The MB87316 has two independent banks and each bank has, at any instant,
	// exactly one writer: the write side drives the bank that is not displayed
	// (`~RSEL`) and the clear-on-read side drives the one that is.  Both fire on
	// `ce_modclk_r` and `rb` decides which is which, so one write port per bank
	// carries both — the mux below is not an arbitration, it is that same mutual
	// exclusion written where the synthesiser can see it.  Keep it merged:
	// written as two separate `memN[...] <= ...` statements, one per process,
	// each array has three ports, which no Cyclone V block RAM has, and Quartus
	// builds both banks out of flip-flops — about 11 700 ALMs and as many
	// registers, 28 % of the device, for 8192 bits.  Merged, each bank fits in
	// one M10K.
	wire       m0_we = ce_modclk_r & ~reset & (rb ? w_we : (rd_loc_v & r_we));
	wire [8:0] m0_wa = rb ? wcnt0  : rd_loc;
	wire [7:0] m0_wd = rb ? w_data : r_data;
	wire       m1_we = ce_modclk_r & ~reset & (rb ? (rd_loc_v & r_we) : w_we);
	wire [8:0] m1_wa = rb ? rd_loc : wcnt1;
	wire [7:0] m1_wd = rb ? r_data : w_data;

	always_ff @(posedge clk) begin
		if (m0_we) mem0[m0_wa] <= m0_wd;
		if (m1_we) mem1[m1_wa] <= m1_wd;
	end

	// =====================================================================
	// Read side — RCLK (MORCLK) rising reads and advances (ph 2 of a count);
	// ACLK (7M) falling publishes the value on DO (ph 4).  See the header.
	// =====================================================================
	logic [7:0] rd_val;
	logic [8:0] rd_loc;
	logic       rd_loc_v;
	always_ff @(posedge clk) begin
		if (reset) begin
			rcnt0    <= 9'd0;
			rcnt1    <= 9'd0;
			rd_val   <= IDLE;
			rd_loc   <= 9'd0;
			rd_loc_v <= 1'b0;
			do_out   <= IDLE;
		end else begin
			// RCLK rising (ph 2): read and advance
			if (ce_morclk_r) begin
				rd_loc_v <= 1'b0;
				if (rb) begin                        // bank 1 is displayed
					if (!r_mor_n)
						rcnt1 <= 9'(RD_ADJ);
					else begin
						rd_val   <= mem1[rcnt1];
						rd_loc   <= rcnt1;
						rd_loc_v <= 1'b1;
						rcnt1    <= rcnt1 + 9'd1;
					end
				end else begin                       // bank 0 is displayed
					if (!r_mor_n)
						rcnt0 <= 9'(RD_ADJ);
					else begin
						rd_val   <= mem0[rcnt0];
						rd_loc   <= rcnt0;
						rd_loc_v <= 1'b1;
						rcnt0    <= rcnt0 + 9'd1;
					end
				end
			end
			// ACLK (7M) falling (ph 4): publish
			if (ce_m7_fall) do_out <= rd_val;
			// DCLK rising (ph 6): clear-on-read.  The display bank's WE is
			// asserted unconditionally with DIE deasserted, and the MB87316
			// writes on DCLK, so the location just read is overwritten with the
			// idle value in the same pixel period.  That write is issued from
			// the merged write port above (`m0_we` / `m1_we` carry exactly
			// `ce_modclk_r & rd_loc_v & r_we` on the displayed bank) so each
			// array keeps one write port.
		end
	end

	assign wcount  = rb ? wcnt0 : wcnt1;
	assign rcount  = rb ? rcnt1 : rcnt0;
	assign wr_stb  = ce_modclk_r & w_we;
	assign wr_addr = wcount;

endmodule
