`timescale 1ns/1ps
//============================================================================
//  Bad Lands address decode -- the GAL 26C (136074-1001), the LS138 24C it
//  enables, and the program-ROM chip/output enables, from schematic sheet 2.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Structure ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_addr_decode.sv`
//  (GPL-3.0, same author); the equations are out of the 136074-1001 fuse map.
//
//  ---- GAL 26C, a GAL16V8 in 16L8 mode, all outputs combinatorial ----------
//  pins 1..8   = /AS, R/W, A13, A14, A15, A16, A17, A23
//  pins 9, 11  = BS0, BS1 from the empty SLAPSTIC socket 28C: in no product
//                term, so there is no SLAPSTIC to implement
//  pin 12      = slapstic /CS, macrocell OE = 0, permanently tri-stated
//  pins 13..19 = /MISC2, /MISC, /EEROM, /CRAM, /RAM, A13S, A14S
//
//  `AS` = "/AS asserted", `WR` = R/W == 0, `RD` = R/W == 1:
//
//    /MISC2 = AS & /A13 & /A14 & /A15 & /A16 & /A17 & A23
//    /MISC  = AS & WR &  A14 &  A15 & /A16 &  A17 & A23              (1)
//           + AS & WR & /A13 & /A14 & /A16 &  A17 & A23              (2)
//           + AS & WR & /A14 & /A15 & /A16 &  A17 & A23              (3)
//           + AS & RD &  A13 & /A14 &  A15 & /A16 &  A17 & A23       (4)
//           + AS & RD &  A14 & /A15 & /A16 &  A17 & A23              (5)
//    /EEROM = AS & /A13 & /A14 & /A15 &  A16 & /A17 & A23
//    /CRAM  = AS & /A13 &  A14 &  A15 &  A16 &  A17 & A23
//    /RAM   =      A13 &  A14 &  A15 &  A16 &  A17 & A23   (no /AS term)
//    A13S   = A13 ,  A14S = A14
//
//  /RAM has no /AS qualifier, so it can be asserted while the bus is idle;
//  GAL 21R takes /AS separately on its pin 3, so do not AND the two here.
//  A14S/A13S are the empty socket's pass-through (o18 = i3, o19 = i4), so
//  0x008000-0x00FFFF is plain ROM.  Read/write direction is enforced by the
//  /MISC product terms: a read of 0xFE0001 asserts nothing, a write of
//  0xFE4001 asserts nothing.  Nothing here looks at /UDS //LDS, so a word
//  access to the even address fires the same strobe as the odd-byte access --
//  which is what the program does for /LETA and /UNLOCK.
//
//  ---- LS138 24C ----------------------------------------------------------
//  G1 (6) = +5 V, /G2A (4) = GND, /G2B (5) = /MISC, C,B,A (3,2,1) = A15,A14,A13
//    Y0 /WDOG    FE0000-FE1FFF  W        Y4 /AUDWR   FE8000-FE9FFF  W  D15:8
//    Y1 /VIRQACK FE2000-FE3FFF  W        Y5 /AUDRD   FEA000-FEBFFF  R  D15:8
//    Y2 /SWITCHES FE4000-FE5FFF R D7:0   Y6 /PFBANK  FEC000-FEDFFF  W  D0
//    Y3 /LETA    FE6000-FE7FFF  R D7:0   Y7 /UNLOCK  FEE000-FEFFFF  W
//  A12:A1 are undecoded inside each 8 KB Y window; /LETA additionally uses
//  A2:A1 as the channel select.
//
//  ---- program ROM --------------------------------------------------------
//  All four 27512s are wired identically except /OE.  ROM address =
//  {A16, A15, A14S, A13S, A12..A1}; /CE = A23 (selected when A23 = 0).
//    20F /OE = A17  -> D15:8, 0x000000-0x01FFFF even bytes
//    27F /OE = A17  -> D7:0 , 0x000000-0x01FFFF odd bytes
//    17F /OE = /A17 -> D15:8, 0x020000-0x03FFFF even
//    24F /OE = /A17 -> D7:0 , 0x020000-0x03FFFF odd
//  A18..A22 are undecoded, so the 256 KB repeats 32 times through 0x7FFFFF.
//  /CE is a pure A23 compare and so asserts for writes too, but writes into
//  ROM space are inert: the parts have no /WE.
//
//  26C and the LS138 are both purely combinatorial, so this module takes no
//  clock.
//============================================================================

module badlands_addr_decode
(
	// ---- 68000 bus (all levels, no clock: this is combinatorial glue) ----
	input  logic        as,        // active-high copy of /AS  (15K F04 3->4)
	input  logic        rw,        // 1 = read, 0 = write
	// A22..A18 reach no product term and no ROM address pin; that is the
	// mirroring described above, not an oversight.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [23:1] a,
	/* verilator lint_on UNUSEDSIGNAL */

	// ---- GAL 26C outputs, active low exactly as the pins are named -------
	output logic        misc2_n,   // pin 13  0xFC0000
	output logic        misc_n,    // pin 14  0xFE0000  (enables the LS138)
	output logic        eerom_n,   // pin 15  0xFD0000
	output logic        cram_n,    // pin 16  0xFFC000
	output logic        ram_n,     // pin 17  0xFFE000  -- no /AS term
	output logic        a13s,      // pin 18  = A13 (slapstic pass-through)
	output logic        a14s,      // pin 19  = A14 (slapstic pass-through)

	// ---- LS138 24C outputs, active low -----------------------------------
	output logic        wdog_n,     // Y0  FE0001  W
	output logic        virqack_n,  // Y1  FE2001  W
	output logic        switches_n, // Y2  FE4001  R  D7:0
	output logic        leta_n,     // Y3  FE6001  R  D7:0, A2:1 = channel
	output logic        audwr_n,    // Y4  FE8000  W  D15:8
	output logic        audrd_n,    // Y5  FEA000  R  D15:8
	output logic        pfbank_n,   // Y6  FEC001  W  D0
	output logic        unlock_n,   // Y7  FEE001  W

	// ---- program ROM -----------------------------------------------------
	output logic        rom_ce_n,   // = A23
	output logic        rom_oe_lo_n,// 20F/27F, /OE = A17   -> 0x000000-0x01FFFF
	output logic        rom_oe_hi_n,// 17F/24F, /OE = /A17  -> 0x020000-0x03FFFF
	output logic [15:0] rom_a       // {A16, A15, A14S, A13S, A12..A1}: a pair of
	                                // 64 K x 8 parts covers 64 K words, so A17 is
	                                // the /OE and never a part address bit
);

	// ---------------- the GAL's own input names ---------------------------
	wire a13 = a[13], a14 = a[14], a15 = a[15];
	wire a16 = a[16], a17 = a[17], a23 = a[23];
	wire wr  = ~rw;
	wire rd  =  rw;

	// ---------------- GAL 26C (136074-1001) -------------------------------
	// Written as the active-low pin: pin = ~(sum of products).
	assign misc2_n = ~( as & ~a13 & ~a14 & ~a15 & ~a16 & ~a17 & a23 );

	assign misc_n  = ~( ( as & wr        &  a14 &  a15 & ~a16 &  a17 & a23 )   // (1)
	                  | ( as & wr & ~a13 & ~a14        & ~a16 &  a17 & a23 )   // (2)
	                  | ( as & wr        & ~a14 & ~a15 & ~a16 &  a17 & a23 )   // (3)
	                  | ( as & rd &  a13 & ~a14 &  a15 & ~a16 &  a17 & a23 )   // (4)
	                  | ( as & rd        &  a14 & ~a15 & ~a16 &  a17 & a23 ) );// (5)

	assign eerom_n = ~( as & ~a13 & ~a14 & ~a15 &  a16 & ~a17 & a23 );
	assign cram_n  = ~( as & ~a13 &  a14 &  a15 &  a16 &  a17 & a23 );

	// No /AS term: a pure address compare.
	assign ram_n   = ~(       a13 &  a14 &  a15 &  a16 &  a17 & a23 );

	assign a13s    = a13;    // o18 = i3, the empty-socket pass-through
	assign a14s    = a14;    // o19 = i4

	// ---------------- LS138 24C -------------------------------------------
	// /G2A = GND, G1 = +5 V, /G2B = /MISC ; C,B,A = A15,A14,A13
	wire [2:0] y_sel = {a15, a14, a13};
	logic [7:0] y_n;
	always_comb begin
		y_n = 8'hFF;
		if (!misc_n) begin
			case (y_sel)
				3'd0: y_n[0] = 1'b0;
				3'd1: y_n[1] = 1'b0;
				3'd2: y_n[2] = 1'b0;
				3'd3: y_n[3] = 1'b0;
				3'd4: y_n[4] = 1'b0;
				3'd5: y_n[5] = 1'b0;
				3'd6: y_n[6] = 1'b0;
				default: y_n[7] = 1'b0;
			endcase
		end
	end

	assign wdog_n     = y_n[0];
	assign virqack_n  = y_n[1];
	assign switches_n = y_n[2];
	assign leta_n     = y_n[3];
	assign audwr_n    = y_n[4];
	assign audrd_n    = y_n[5];
	assign pfbank_n   = y_n[6];
	assign unlock_n   = y_n[7];

	// ---------------- program ROM -----------------------------------------
	// The /OE pin of 20F/27F is driven with A17 itself, so those two parts are
	// enabled when A17 = 0; 17F/24F get /A17 (15K F04 1->2) and are enabled
	// when A17 = 1.
	assign rom_ce_n    =  a23;          // /CE = A23: selected when A23 = 0
	assign rom_oe_lo_n =  a17;          // 20F/27F -> 0x000000-0x01FFFF
	assign rom_oe_hi_n = ~a17;          // 17F/24F -> 0x020000-0x03FFFF
	assign rom_a       = {a[16], a[15], a14s, a13s, a[12:1]};

endmodule
