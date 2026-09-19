`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- YM2151 9D + YM3012 8E, through jt51.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  Ported from the Blasteroids MiSTer core's `blstroid_ym.sv` (GPL-3.0, same
//  author): the jt51 instantiation and the fast-6502 -> slow-YM write bridge
//  are unchanged.  The wiring is Bad Lands' own and differs from the JSA board
//  in two ways: /IRQ (pin 2) is a bare stub and CT1/CT2 (pins 8, 9) are stubs
//  too, so there is no external port and no stereo gate to switch.
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  YM2151 9D, schematic sheet 5:
//     pin 24  ØM   = 1H = 3.579545 MHz          -> `cen`
//     pin  4  A0   = SA0
//     pin  3  /IC  = /YAMRES (10E bit 0)        -> 0 holds the chip in reset
//     pin  5  /WR  = /SWR    pin 6 /RD = /SRD    pin 7 /CS = /YAM
//     pin 18..10   D7..D0 = SD7..SD0
//     pin  2  /IRQ, pins 8 and 9 CT1, CT2 -- bare stubs, not connected
//     pins 23, 21, 20, 19  Ø1, SO, SH1, SH2 -> 8E YM3012 CK0, DATA, SAM1, SAM2
//
//  The firmware polls the status register instead of using the interrupt
//  ($41C1: lda $2001 / and #$01 for timer A; $4FF6: bit $2001 / bpl for BUSY).
//  jt51's `dout` is that register and is driven whenever the bus asks for it,
//  so a read of $2001 -- or of any mirror, since A0 is the only decoded low
//  address bit (mirror $07FE) -- returns BUSY on bit 7, timer flags on bits 1:0.
//
//  YM3012 8E: ICL (7) = /YAMRES; CH1 (10) and CH2 (9) each carry a 1 nF cap and
//  a 5E LM324 unity buffer into the summing node through R68 and R67, both
//  18 K, i.e. exactly equal weight.  Which of jt51's `xleft`/`xright` is the
//  part's SH1 and which SH2 is therefore unobservable: the two are summed with
//  identical coefficients into one mono output, so swapping them cannot change
//  a sample.
//
//  Write bridge: the 6502 writes at bus speed but jt51 captures
//  `write = !cs_n & !wr_n` on `cen_p1`.  Here `cen_p1` IS `ce_2h` and the /SWR
//  strobe is already ce_2h-qualified, so the bridge never has to hold anything;
//  it is kept because it costs one register.  Zero write latency matters: the
//  YM2151 sets BUSY when it accepts the write and the firmware spins on that
//  flag, so a one-cen_p1 (559 ns) delay would move the poll window by a whole
//  6502 cycle.  The real 9D latches on /WR + /CS during the bus cycle.
//
//  Clock enables: clk_sys = 57.272724 MHz = 4 x the 14.31818 MHz crystal, so
//      ce_1h = clk_sys/16 = 3.579545  MHz -> cen     (the ØM pin)
//      ce_2h = clk_sys/32 = 1.7897725 MHz -> cen_p1  (the 6502's Ø0)
//  come from one free-running counter and ce_2h is a strict subset of ce_1h --
//  the "cen_p1 is every other cen, aligned with it" relationship jt51 wants.
//============================================================================

module badlands_ym
(
	input  logic        clk,
	input  logic        cen,        // ce_1h = 3.579545 MHz   (YM2151 pin 24)
	input  logic        cen_p1,     // ce_2h = 1.7897725 MHz, aligned with cen
	input  logic        yamres_n,   // /YAMRES from 10E bit 0 -> YM2151 /IC

	// bus side (from badlands_snd_bus)
	input  logic        ym_cs,      // /YAM asserted
	input  logic        ym_a0,      // SA0
	input  logic        ym_we,      // /SWR & /YAM
	input  logic  [7:0] ym_dout,    // 6502 -> YM
	output logic  [7:0] ym_din,     // YM status -> 6502 (bit 7 = BUSY)

	// audio: CH1 / CH2 into the analogue summing node, equal weight
	output logic        sample,
	output logic signed [15:0] aud_ch1,
	output logic signed [15:0] aud_ch2
);

	// ---- write bridge -------------------------------------------------------
	logic       wr_pend, wr_a0;
	logic [7:0] wr_d;
	initial begin wr_pend = 1'b0; wr_a0 = 1'b0; wr_d = 8'h00; end
	wire        wr_cap = ym_cs & ym_we;
	// `always @(posedge clk)` and not `always_ff`: these registers have no reset,
	// so their power-up value comes from the `initial` above -- and IEEE 1800
	// 9.2.2.4 forbids an always_ff variable being written by another process.
	always @(posedge clk) begin
		if (~yamres_n) begin
			wr_pend <= 1'b0;
		end else if (wr_cap & ~cen_p1) begin   // strobe missed cen_p1: hold it
			wr_pend <= 1'b1; wr_a0 <= ym_a0; wr_d <= ym_dout;
		end else if (cen_p1) begin
			wr_pend <= 1'b0;                   // jt51 consumed it on cen_p1
		end
	end

	wire        wr_go   = wr_cap | wr_pend;
	wire        jt51_a0 = wr_cap ? ym_a0   : wr_a0;
	wire  [7:0] jt51_d  = wr_cap ? ym_dout : wr_d;

	wire irq_n_nc, ct1_nc, ct2_nc;
	wire signed [15:0] lo_l_nc, lo_r_nc;

	jt51 u_jt51 (
		.rst    (~yamres_n),      // /IC (pin 3) -- the only reset the chip has
		.clk    (clk),
		.cen    (cen),
		.cen_p1 (cen_p1),
		.cs_n   (~wr_go),
		.wr_n   (~wr_go),
		.a0     (jt51_a0),
		.din    (jt51_d),
		.dout   (ym_din),
		.ct1    (ct1_nc),
		.ct2    (ct2_nc),
		.irq_n  (irq_n_nc),
		.sample (sample),
		.left   (lo_l_nc),
		.right  (lo_r_nc),
		.xleft  (aud_ch1),
		.xright (aud_ch2)
	);

	// /IRQ (pin 2), CT1 (8) and CT2 (9) are bare stubs on sheet 5: the chip has
	// no path to the 6502's /IRQ and no external port bits.  They are read and
	// discarded here so the disconnection is visible rather than implied.
	// jt51's low-resolution `left`/`right` are unused; the board's YM3012 gets
	// the full-resolution serial stream.
	wire _unused_ym = &{ 1'b0, irq_n_nc, ct1_nc, ct2_nc, lo_l_nc, lo_r_nc };

endmodule
