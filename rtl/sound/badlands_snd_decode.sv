`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- 6502 sound-section address decode:
//  GAL 12E (136074-1005) + the LS138 at 10F, as schematic sheet 5 draws them.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  Structure (module split, port style, the "raw active-high nets plus
//  Phi2-qualified strobes" convention) follows the Blasteroids MiSTer core's
//  `blstroid_jsa_decode.sv` (GPL-3.0, same author).  Every equation below is
//  Bad Lands' own: this is not a JSA board.
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  GAL16V8 at 12E in 16L8 mode: all eight outputs combinatorial, active low,
//  .oe = vcc.  Pure combinational logic -- no state, no clock.
//
//     pin 1,2      SR/W, PH2                 pin 12  /ROM   -> 27512 9C /CE
//     pin 3,4      BA12, BA13 (10E Q2, Q1)   pin 13  /YAM   -> YM2151 9D /CS
//     pin 5..8,11  SA9, SA11, SA12, SA13,    pin 14  /REST  -> LS138 10F /G2B
//                  SA14; SA15                pin 15  /SRAM  -> 6264 9B /CS1
//                                            pin 16  SA13B  -> 27512 9C A13
//                                            pin 17  SA12B  -> 27512 9C A12
//                                            pin 18  /SWR   -> 9B /WE, YM /WR
//                                            pin 19  /SRD   -> 9C /OE, 9B /OE,
//                                                              YM /RD
//
//  The equations from the 136074-1005 fuse dump, verbatim.  `B` = the
//  $2800-$2FFF block, `R` = SR/W, `W` = !SR/W:
//
//    B     = SA11 & !SA12 & SA13 & !SA14 & !SA15
//    /ROM  = R & SA15  +  R & SA14  +  R & SA13 & SA12
//    /YAM  = !SA11 & !SA12 & SA13 & !SA14 & !SA15
//    /REST = PH2 & SA11 & !SA12 & SA13 & !SA14 & !SA15
//    /SRAM = !SA13 & !SA14 & !SA15
//    SA13B = SA13 & ( BA13 | !SA12 | SA14 | SA15 )
//    SA12B = SA12 & ( BA12 | !SA13 | SA14 | SA15 )
//    /SWR  = W & PH2 & !SA13 & !SA14 & !SA15
//          + W & PH2 & !SA11 & !SA12 & SA13 & !SA14 & !SA15
//    /SRD  = (R & PH2 & !B) + ( B & ( !PH2 | (R & SA9) | (W & !SA9) ) )
//
//  Two of these need care:
//   * The literal /SRD term inside `B` is odd but inert: all three /SRD loads
//     (9C /OE, 9B /OE, YM /RD) are gated by their own chip selects and none is
//     asserted inside $2800-$2FFF, so the model used here is `SR/W & PH2`.
//     `srd` is that model, `srd_gal` the literal dump, so the two can be
//     compared.
//   * /ROM is not PH2-qualified.  The 27512's /CE follows SR/W and the address
//     directly; only its /OE (/SRD) carries PH2.  `rom_ce` below is therefore
//     the raw address+direction decode, and a ROM read must be gated with PH2
//     (i.e. with `srd`), which badlands_snd_bus does.
//
//  /SWR has only the two product terms above, so a write into $3000-$FFFF, into
//  the $2C00-$2FFF hole or onto an LS138 strobe cannot disturb a memory.
//
//  LS138 10F:
//     G1 (6) = +5 V        /G2B (5) = /REST        /G2A (4) = SA10
//     C,B,A (3,2,1) = SA9, SA2, SA1
//
//     strobe window = PH2 & !SA15 & !SA14 & SA13 & !SA12 & SA11 & !SA10
//                   = PH2 & $2800-$2BFF
//     select        = { SA9, SA2, SA1 }
//     don't care    = SA8..SA3 and SA0     ->  mirror mask $01F9
//
//     Y0 $2800 unused stub   Y4 $2A00 unused stub
//     Y1 $2802 /RDP          Y5 $2A02 /WRP
//     Y2 $2804 /RDIO         Y6 $2A04 /WRIO
//     Y3 $2806 /IRQACK       Y7 $2A06 unused stub
//
//  Three differences from the JSA board this file's structure came from:
//   1. No R/W qualification.  The JSA's LS138 has G1 = /SRD; Bad Lands ties G1
//      to +5 V, so SA9 alone separates the "read" group from the "write" group,
//      by software convention only.  /IRQACK therefore acks on a read or a
//      write, which is what the firmware needs (it only ever reads $280E); a
//      write to $2802/$2804 or a read of $2A02/$2A04 would be real bus
//      contention, and the firmware never does either.
//   2. /G2A is SA10 and /G2B is /REST (the JSA has them the other way round),
//      so SA10 must be 0: $2C00-$2FFF decodes nothing at all -- not the LS138
//      (SA10 = 1) and not /YAM (which needs !SA11).  MAME's mirror $05F9
//      wrongly folds A10 in.
//   3. Y0, Y4 and Y7 are unconnected stubs.  The firmware writes $3C to $2A00
//      and $2A06 ($42BA/$42BD, $59AA/$59AD); those are no-ops.
//
//  PH2: a 6502 bus cycle is one `ce_2h` clock enable, so `ph2` IS `ce_2h`.
//  /SRAM, /YAM, `rom_ce`, SA13B and SA12B deliberately do not carry it -- they
//  are pure address (and, for /ROM, direction) decodes, so they stay valid for
//  the whole cycle and can drive the synchronous memories, whose reads are
//  registered a clock ahead of `ce_2h`.
//============================================================================

module badlands_snd_decode
(
	// ---- 6502 (14C) ----
	input  logic [15:0] sa,          // SA15..SA0
	input  logic        srw,         // SR/W, 1 = read      -- 12E pin 1
	input  logic        ph2,         // PH2 (6502 pin 39)   -- 12E pin 2

	// ---- 10E LS174 bank bits ----
	input  logic        ba13,        // 10E Q1              -- 12E pin 4
	input  logic        ba12,        // 10E Q2              -- 12E pin 3

	// ---- GAL 12E outputs, in active-high "asserted" form ----
	output logic        rom_ce,      // /ROM  (pin 12) -- not PH2-qualified
	output logic        yam,         // /YAM  (pin 13)
	output logic        rest,        // /REST (pin 14) -- PH2 & $2800-$2FFF
	output logic        sram,        // /SRAM (pin 15)
	output logic        sa13b,       // SA13B (pin 16) -- 9C A13, positive sense
	output logic        sa12b,       // SA12B (pin 17) -- 9C A12, positive sense
	output logic        swr,         // /SWR  (pin 18)
	output logic        srd,         // /SRD  (pin 19) -- the model
	output logic        srd_gal,     // /SRD  (pin 19) -- the literal fuse dump

	// ---- address-only windows (no PH2, no direction) ----
	output logic        sel_ram,     // $0000-$1FFF  -> 6264 9B
	output logic        sel_ym,      // $2000-$27FF  -> YM2151 9D  (= yam)
	output logic        sel_bank,    // $3000-$3FFF  -> 9C, A13/A12 = BA13/BA12
	output logic        sel_romfix,  // $4000-$FFFF  -> 9C straight through
	output logic        sel_hole,    // $2C00-$2FFF  -> nothing decodes

	// ---- LS138 10F outputs, active-high, PH2-qualified, no direction term ---
	output logic        sel_y0,      // Y0 $2800  unconnected stub
	output logic        sel_rdp,     // Y1 $2802  /RDP     15F /OE
	output logic        sel_rdio,    // Y2 $2804  /RDIO    16C /1G,/2G
	output logic        sel_irqack,  // Y3 $2806  /IRQACK  14E /CLR
	output logic        sel_y4,      // Y4 $2A00  unconnected stub
	output logic        sel_wrp,     // Y5 $2A02  /WRP     13F CP, 15J.1 CLK
	output logic        sel_wrio,    // Y6 $2A04  /WRIO    10E CP
	output logic        sel_y7,      // Y7 $2A06  unconnected stub

	// ---- misc ----
	output logic        ym_a0,       // YM2151 pin 4 = SA0
	output logic [15:0] rom_addr     // 9C A15..A0 = {SA15,SA14,SA13B,SA12B,SA11:0}
);

	// ---- the GAL's own literal terms ---------------------------------------
	// `B` is the $2800-$2FFF block used by /REST and by the literal /SRD.
	wire r = srw;
	wire w = ~srw;
	wire b = sa[11] & ~sa[12] & sa[13] & ~sa[14] & ~sa[15];

	assign rom_ce  = (r & sa[15]) | (r & sa[14]) | (r & sa[13] & sa[12]);
	assign yam     = ~sa[11] & ~sa[12] &  sa[13] & ~sa[14] & ~sa[15];
	assign rest    = ph2 & b;
	assign sram    = ~sa[13] & ~sa[14] & ~sa[15];
	assign sa13b   = sa[13] & ( ba13 | ~sa[12] | sa[14] | sa[15] );
	assign sa12b   = sa[12] & ( ba12 | ~sa[13] | sa[14] | sa[15] );
	assign swr     = ( w & ph2 & ~sa[13] & ~sa[14] & ~sa[15] )
	               | ( w & ph2 & ~sa[11] & ~sa[12] & sa[13] & ~sa[14] & ~sa[15] );

	// The literal dump and the model.  They differ only inside `B`, where no
	// /SRD load is selected.
	assign srd_gal = ( r & ph2 & ~b ) | ( b & ( ~ph2 | (r & sa[9]) | (w & ~sa[9]) ) );
	assign srd     = r & ph2;

	// ---- the address windows the memories see ------------------------------
	assign sel_ram    = sram;                       // $0000-$1FFF
	assign sel_ym     = yam;                        // $2000-$27FF (mirror $07FE)
	assign sel_bank   = (sa[15:12] == 4'h3);        // $3000-$3FFF
	assign sel_romfix = (sa[15:14] != 2'b00);       // $4000-$FFFF
	assign sel_hole   =  sa[11] & ~sa[12] & sa[13] & ~sa[14] & ~sa[15] & sa[10];

	// ---- LS138 10F ---------------------------------------------------------
	// G1 = +5 V (no direction term), /G2B = /REST, /G2A = SA10.
	wire       en138 = rest & ~sa[10];
	wire [2:0] s138  = { sa[9], sa[2], sa[1] };
	wire [7:0] y138  = en138 ? (8'h01 << s138) : 8'h00;

	assign sel_y0     = y138[0];
	assign sel_rdp    = y138[1];
	assign sel_rdio   = y138[2];
	assign sel_irqack = y138[3];
	assign sel_y4     = y138[4];
	assign sel_wrp    = y138[5];
	assign sel_wrio   = y138[6];
	assign sel_y7     = y138[7];

	assign ym_a0    = sa[0];
	assign rom_addr = { sa[15], sa[14], sa13b, sa12b, sa[11:0] };

	// Nothing inside the LS138 window decodes SA8..SA3 or SA0 (the $01F9
	// mirror), which is why the firmware's $280A / $280C / $280E work.  They
	// still reach the memories through `sa`.
	wire _unused_dec = &{ 1'b0, sa[8:3] };

endmodule
