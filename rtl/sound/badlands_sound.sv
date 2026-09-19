`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- the complete on-board sound section.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  Not a JSA board: Bad Lands has no separate audio PCB.  The 6502A (14C), the
//  GAL 12E decode, the LS138 10F, the 6264 9B, the 27512 9C, the 10E/14E/16C
//  glue, the YM2151 9D, the YM3012 8E and the whole LM324/TDA2030 tail are on
//  the game PCB, A047193-01, schematic sheet 5.  The Blasteroids / Xybots /
//  Vindicators cores' `*_jsa.sv` was the structural template (GPL-3.0, same
//  author) and nothing else: there is no SCOM, no periodic-IRQ divider, no
//  POKEY, no TMS5220, no /MIX latch and no stereo.
//
//  Submodules:
//    badlands_snd_bus     6502A 14C (T65) + GAL 12E + LS138 10F + 6264 9B +
//                         27512 9C with the {BA13,BA12} $3000-$3FFF window
//    badlands_snd_decode  GAL 136074-1005 and the LS138, verbatim
//    badlands_snd_io      10E LS174, 14E 74F74, 16C LS240
//    badlands_mailbox     15F / 13F LS374 + the two 15J 74F74 flags
//    badlands_ym          YM2151 9D through jt51, YM3012 8E pairing
//    badlands_snd_filter  the 5E/2E/TDA2030 analogue chain, digitised
//
//  Clock enables:
//    clk    = clk_sys = 57.272724 MHz (4 x the 14.31818 MHz crystal)
//    ce_14m = /4  14.318181 MHz
//    ce_1h  = /16  3.579545 MHz   YM2151 ØM (pin 24)
//    ce_2h  = /32  1.7897725 MHz  6502 Ø0 (pin 37) -- and PH2 for the decode
//  ce_2h must be a strict subset of ce_1h (jt51's cen / cen_p1 relationship).
//  `ce_14m` is accepted for interface symmetry with the rest of the core and is
//  deliberately unused here: nothing in the sound section runs at 14 MHz.  The
//  analogue filter runs on ce_2h.
//
//  Reset: there is exactly one reset into this block, `audres_n`, and the main
//  board must drive it with the whole net
//      /AUDRESD = R/W + /MISC2        (17J F32)  -- low only on a write to FC0000
//      /AUDRES  = /RES AND /AUDRESD   (19J LS08)
//  so it is asserted by the power-on reset and by the 0xFC0000 write.  It
//  reaches exactly two loads: the 6502's /RES (pin 40) and the 10E LS174's /MR.
//  It does not clear the mailbox flags (15J) or the 6502 IRQ latch (14E), so a
//  command can survive an /AUDRES pulse.  There is no other reset port; adding
//  one would be wrong.
//
//  Note the crossed JAMMA coin-counter naming: `cctr2` (10E bit 5) is JAMMA
//  "COIN CTR 1" and `cctr1` (bit 4) is "COIN CTR 2", and R6 = 0 ohm ties both
//  JAMMA pins into one node (`cctr_wired_or`).
//
//  The board is mono -- JAM-10 SPKR+ and the pre-volume JAM-11 AUDIO+ tap carry
//  one signal -- so `audio_l` and `audio_r` are identical by construction.
//============================================================================

module badlands_sound
#(
	// Test only.  0 = the board's /RDIO byte, which is what the core ships.
	// 1 = MAME's D7/D6 senses, so the two can be compared; see
	// badlands_snd_io.sv for which bits differ and why only D7 matters.
	parameter bit MAME_RDIO_POLARITY = 1'b0
)
(
	input  logic        clk,
	input  logic        ce_2h,          // 6502 Ø0 = 1.789772 MHz
	input  logic        ce_1h,          // YM2151 ØM = 3.579545 MHz
	input  logic        ce_14m,         // 14.318181 MHz -- see the header
	input  logic        audres_n,       // /RES AND /AUDRESD

	// ---- raster taps for the 14E interrupt latch ----
	input  logic        vblank,
	input  logic        v32,            // the SOS-2 V counter's bit 5

	// ---- board inputs ----
	input  logic        test_n,         // /TEST, low = self-test switch ON
	input  logic        coin1,          // JAM-16, high = closed
	input  logic        coin2,          // JAM-T,  high = closed

	// ---- 68000 mailbox ----
	input  logic        audwr_stb,      // write 0xFE8000
	input  logic  [7:0] audwr_data,     // D15:8
	input  logic        audrd_stb,      // read  0xFEA000
	output logic  [7:0] audrd_data,
	output logic        audfull,        // -> D8 at a 0xFC0000 read
	output logic        audirq_n,       // -> 68000 IRQ2

	// ---- coin counters ----
	output logic        cctr1,          // 10E bit 4 -> JAMMA "COIN CTR 2"
	output logic        cctr2,          // 10E bit 5 -> JAMMA "COIN CTR 1"

	// ---- sound ROM load port (download offset 0x040000) ----
	input  logic        rom_wr,
	input  logic [15:0] rom_addr,
	input  logic  [7:0] rom_data,

	// ---- audio (mono: both outputs carry the same sample) ----
	output logic signed [15:0] audio_l,
	output logic signed [15:0] audio_r
);

	// ---- mailboxes (15F / 13F / 15J) ---------------------------------------
	wire [7:0] rdp_data;
	wire       mainirq_n;
	wire       sel_rdp, sel_rdio, sel_irqack, sel_wrp, sel_wrio;
	wire       sel_y0, sel_y4, sel_y7;
	wire [7:0] cpu_dout;

	badlands_mailbox u_mbox (
		.clk(clk),
		.audwr_stb(audwr_stb), .audwr_data(audwr_data),
		.audrd_stb(audrd_stb), .audrd_data(audrd_data),
		.audfull(audfull), .audirq_n(audirq_n),
		.rdp_stb(sel_rdp), .wrp_stb(sel_wrp), .wrp_data(cpu_dout),
		.rdp_data(rdp_data), .mainirq_n(mainirq_n) );

	// ---- 10E / 14E / 16C ----------------------------------------------------
	wire [1:0] bank;
	wire       yamres_n, irq_n, cctr_wired_or, latch_q5;
	wire [7:0] rdio_data;

	badlands_snd_io u_io (
		.clk(clk),
		.audres_n(audres_n), .wrio_stb(sel_wrio), .wrio_data(cpu_dout),
		.vblank(vblank), .v32(v32), .irqack(sel_irqack),
		.test_n(test_n),
		.cmd_pending(~mainirq_n), .resp_pending(~audirq_n),
		.coin1(coin1), .coin2(coin2),
		.mame_polarity(MAME_RDIO_POLARITY),  // 0 = the board; see the header
		.bank(bank), .cctr2(cctr2), .cctr1(cctr1),
		.cctr_wired_or(cctr_wired_or), .latch_q5(latch_q5),
		.yamres_n(yamres_n), .irq_n(irq_n), .rdio_data(rdio_data) );

	// ---- 6502 + decode + memories ------------------------------------------
	wire        ym_cs, ym_a0, ym_we, ym_rd;
	wire [7:0]  ym_dout, ym_din;
	wire [15:0] cpu_addr;
	wire [7:0]  cpu_din;
	wire        cpu_rnw, cpu_sync;

	badlands_snd_bus u_bus (
		.clk(clk), .ce_2h(ce_2h),
		.audres_n(audres_n), .irq_n(irq_n), .nmi_n(mainirq_n),
		.bank(bank),
		.rdp_data(rdp_data), .rdio_data(rdio_data),
		.ym_cs(ym_cs), .ym_a0(ym_a0), .ym_we(ym_we), .ym_rd(ym_rd),
		.ym_dout(ym_dout), .ym_din(ym_din),
		.sel_rdp(sel_rdp), .sel_rdio(sel_rdio), .sel_irqack(sel_irqack),
		.sel_wrp(sel_wrp), .sel_wrio(sel_wrio),
		.sel_y0(sel_y0), .sel_y4(sel_y4), .sel_y7(sel_y7),
		.rom_wr(rom_wr), .rom_wr_addr(rom_addr), .rom_wr_data(rom_data),
		.cpu_addr(cpu_addr), .cpu_dout(cpu_dout), .cpu_din(cpu_din),
		.cpu_rnw(cpu_rnw), .cpu_sync(cpu_sync) );

	// ---- YM2151 9D + YM3012 8E ---------------------------------------------
	wire signed [15:0] ym_ch1, ym_ch2;
	wire               ym_sample;
	badlands_ym u_ym (
		.clk(clk), .cen(ce_1h), .cen_p1(ce_2h), .yamres_n(yamres_n),
		.ym_cs(ym_cs), .ym_a0(ym_a0), .ym_we(ym_we),
		.ym_dout(ym_dout), .ym_din(ym_din),
		.sample(ym_sample), .aud_ch1(ym_ch1), .aud_ch2(ym_ch2) );

	// ---- the analogue chain -------------------------------------------------
	wire signed [15:0] aud;
	wire               aud_clipped;
	badlands_snd_filter u_filt (
		.clk(clk), .ce(ce_2h), .bypass(1'b0),
		.ch1(ym_ch1), .ch2(ym_ch2), .out(aud), .clipped(aud_clipped) );

	// JAM-10 SPKR+ (post-volume, through the TDA2030) and JAM-11 AUDIO+
	// (pre-volume, R3 + C5 off 2E pin 8) carry one signal.
	assign audio_l = aud;
	assign audio_r = aud;

	// Nets that exist on the board but reach nothing here:
	//  * ym_rd     -- the YM status comes back through the same read mux as
	//                 every other source, so the strobe itself is inert;
	//  * sel_rdio  -- /RDIO gates the 16C LS240's output drivers, which
	//                 badlands_snd_bus already applies inside its read mux;
	//                 the byte itself comes in on `rdio_data`;
	//  * sel_y0/y4/y7 -- LS138 outputs Y0, Y4 and Y7 are unconnected stubs on
	//                 sheet 5.  The firmware writes $3C to $2A00 and $2A06
	//                 ($42BA/$42BD, $59AA/$59AD); those are no-ops.
	//  * cctr_wired_or -- the R6 = 0 ohm node.  `cctr1`/`cctr2` are exported
	//                 separately so the OSD can show either.
	//  * latch_q5  -- 10E Q5: latched by the part, output pin not connected.
	//  * cpu_addr / cpu_din / cpu_rnw / cpu_sync -- bus observability.
	//  * ym_sample, aud_clipped -- diagnostics; `clipped` never fires for a
	//                 YM-sourced signal (see badlands_snd_filter).
	//  * ce_14m    -- nothing in the sound section runs at 14 MHz.
	wire _unused_snd = &{ 1'b0, ce_14m, ym_rd, sel_rdio, sel_y0, sel_y4, sel_y7,
	                      cctr_wired_or, latch_q5, ym_sample, aud_clipped,
	                      cpu_addr, cpu_din, cpu_rnw, cpu_sync };

endmodule
