`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- the three parts hanging off the LS138 10F:
//  the 10E LS174 control latch ($2A04 /WRIO), the 14E 74F74 interrupt latch
//  (`VBLANK | 32V`, acked at $2806 /IRQACK) and the 16C LS240 input port
//  ($2804 /RDIO), from schematic sheet 5.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  Module shape and port style follow the Blasteroids MiSTer core's
//  `blstroid_jsa_io.sv` (GPL-3.0, same author); nothing else transfers.  Bad
//  Lands has no JSA periodic-IRQ divider, no /POR on the output latch, and a
//  different /RDIO bit map.
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  10E LS174, the sound control latch.  CP (9) = /WRIO (rising edge, i.e. the
//  falling edge of PH2, while the write data is still valid); /MR (1) =
//  /AUDRES (asynchronous).
//
//    write bit  10E    net        effect
//      SD7      D1->Q1 BA13       sound-ROM bank MSB
//      SD6      D2->Q2 BA12       sound-ROM bank LSB
//      SD5      D3->Q3 CCTR2      coin-counter driver Q3 -> JAM-8 "COIN CTR 1"
//      SD4      D4->Q4 CCTR1      coin-counter driver Q2 -> JAM-J "COIN CTR 2"
//      SD3, SD2  --    --         not wired to any D input
//      SD1      D5->Q5 --         latched, output pin unconnected
//      SD0      D6->Q6 /YAMRES    YM2151 /IC -- 0 = the YM is held in reset
//
//  /AUDRES clears the whole latch: bank 0, both counters off, YM2151 held in
//  reset.  The firmware's first instructions pulse it deliberately ($4005:
//  lda #$01 / sta $2a04 / and #$fe / sta $2a04 / ora #$01 / sta $2a04).
//
//  Note the cross-over and the 0 ohm link: CCTR2 (bit 5) drives JAMMA COIN
//  CTR 1 and CCTR1 (bit 4) drives COIN CTR 2, and R6 = 0 ohm ties JAM-8 and
//  JAM-J into one node so either pulse drives both outputs.  The crossed names
//  are kept on the ports so nobody "fixes" them the wrong way.
//
//  14E 74F74 half 2, the 6502 interrupt:
//     17J F32 gate 1: pin 1 = VBLANK, pin 2 = 32V, pin 3 -> 14E pin 11
//     14E:  D(12) = +5 V,  /PRE(10) = +5 V,  CLK(11) = 17J.3,
//           /CLR(13) = /IRQACK,  Q(9) = stub,  /Q(8) = 6502 /IRQ
//   * every rising edge of (VBLANK | 32V) asserts /IRQ;
//   * /CLR = /IRQACK is asynchronous and level-sensitive, and on a 74F74 the
//     clear wins a simultaneous clock -- modelled below by clearing last;
//   * there is no reset on 14E: neither /AUDRES nor the power-on reset reaches
//     pins 10 or 13, so the register carries no reset term here.  The firmware
//     acks once during boot ($4024 -> $41B9: lda $280E);
//   * /IRQACK is a read or a write strobe (the LS138 has no direction term),
//     which is what the firmware needs -- it only ever reads $280E.
//
//  Rate, with V = 0 at the first displayed line and VBLANK high for V >= 240:
//     32V high        V = 32-63, 96-127, 160-191, 224-255
//     VBLANK | 32V    high 32-63, 96-127, 160-191, 224-(V_TOTAL-1)
//     rising edges    V = 32, 96, 160, 224  ->  4 per frame, ~4.07 ms apart
//  The VBLANK rising edge at V = 240 falls inside the 224-255 window of 32V, so
//  it adds no edge -- true for V_TOTAL = 262 and 263 alike.  MAME asserts at
//  (scanline % 64) == 0 && scanline < 240, i.e. V = 0, 64, 128, 192: the same
//  count, 32 lines out of phase.
//
//  16C is a 74LS240, an inverting octal buffer, with both /1G (1) and /2G (19)
//  tied to /RDIO, so a logically asserted (low) input reads back as 1:
//    SD7 = NOT(/TEST)    1 while the self-test switch is ON
//    SD6 = NOT(/MAINIRQ) 1 while a main->sound command is pending
//    SD5 = NOT(/AUDIRQ)  1 while a sound->main response is pending
//    SD4 = NOT(/TEST)    the same node as SD7
//    SD3 = 0             its LS240 input is +5 V
//    SD2 = 0             the same +5 V node
//    SD1 = COIN 1        JAM-16, left chute, 1 while the switch is closed
//    SD0 = COIN 2        JAM-T,  right chute, 1 while the switch is closed
//
//  MAME differs from the board in exactly two of these bits and agrees in the
//  other six.  D7 is the one that changes what the 6502 does: the inverting
//  LS240 and the game ROM agree that D7 = 1 with the self-test switch ON
//  ($4016: lda $280c / and #$80 / bne $402a runs the RAM+ROM test only when
//  bit 7 is 1, and the manual says that test runs only with self-test on at
//  power-up), whereas MAME returns 0xC0 with self-test OFF -- so MAME runs the
//  6502 RAM/ROM test during normal play and skips it in self test.  D6 is
//  inverted in MAME too, but the firmware never reads it, so that one is inert.
//  D3 and D2 read 0, not 1, and MAME agrees.  This module implements the board;
//  `mame_polarity` reproduces MAME's D7/D6 senses for comparison and the core
//  ties it to 0.
//============================================================================

module badlands_snd_io
(
	input  logic        clk,

	// ---- 10E LS174 ----
	input  logic        audres_n,      // /AUDRES -> 10E /MR (asynchronous)
	input  logic        wrio_stb,      // /WRIO ($2A04), one ce_2h clock
	input  logic  [7:0] wrio_data,     // SD7:0 at the write

	// ---- 14E 74F74 ----
	input  logic        vblank,        // 17J F32 pin 1, active high
	input  logic        v32,           // 17J F32 pin 2, the V counter's bit 5
	input  logic        irqack,        // /IRQACK ($2806), READ or WRITE

	// ---- 16C LS240 sources ----
	input  logic        test_n,        // /TEST: LOW while the switch is ON
	input  logic        cmd_pending,   // NOT(/MAINIRQ) from 15J half 2
	input  logic        resp_pending,  // NOT(/AUDIRQ)  from 15J half 1
	input  logic        coin1,         // JAM-16, active HIGH = closed
	input  logic        coin2,         // JAM-T,  active HIGH = closed
	input  logic        mame_polarity, // test only: MAME's D7/D6 senses

	// ---- outputs ----
	output logic  [1:0] bank,          // {BA13, BA12} -> GAL 12E pins 4, 3
	output logic        cctr2,         // 10E Q3, bit 5 -> JAMMA COIN CTR 1
	output logic        cctr1,         // 10E Q4, bit 4 -> JAMMA COIN CTR 2
	output logic        cctr_wired_or, // the R6 = 0 ohm node
	output logic        latch_q5,      // 10E Q5, bit 1 -- pin unconnected
	output logic        yamres_n,      // 10E Q6, bit 0 -> YM2151 /IC
	output logic        irq_n,         // 14E /Q -> 6502 /IRQ
	output logic  [7:0] rdio_data      // the assembled $2804 byte
);

	// ---- 10E LS174 ---------------------------------------------------------
	// /MR is asynchronous on the part; here it is a synchronous clear, which is
	// indistinguishable at this clock rate (/AUDRES is at least 1.26 us wide
	// against a 17.5 ns clk_sys period).
	always_ff @(posedge clk) begin
		if (!audres_n) begin
			bank <= 2'd0; cctr2 <= 1'b0; cctr1 <= 1'b0;
			latch_q5 <= 1'b0; yamres_n <= 1'b0;
		end else if (wrio_stb) begin
			bank     <= wrio_data[7:6];   // D1->Q1 = BA13, D2->Q2 = BA12
			cctr2    <= wrio_data[5];
			cctr1    <= wrio_data[4];
			latch_q5 <= wrio_data[1];     // D5->Q5, output pin not connected
			yamres_n <= wrio_data[0];     // D6->Q6 = /YAMRES
		end
	end

	// R6 = 0 ohm ties JAM-8 and JAM-J into one node, so either counter pulse
	// drives both JAMMA outputs.
	assign cctr_wired_or = cctr1 | cctr2;

	// wrio_data[3:2] reach no D input on 10E -- they are not wired to the part.
	wire _unused_wrio = &{ 1'b0, wrio_data[3:2] };

	// ---- 14E 74F74 half 2 --------------------------------------------------
	// No reset on the part; the power-up value 0 is an FPGA fact, not a board one.
	logic irq_q, src_d;
	initial begin irq_q = 1'b0; src_d = 1'b0; end
	wire  src    = vblank | v32;
	// `always @(posedge clk)` and not `always_ff`: these registers have no reset,
	// so their power-up value comes from the `initial` above -- and IEEE 1800
	// 9.2.2.4 forbids an always_ff variable being written by another process.
	always @(posedge clk) begin
		logic q;
		q = irq_q;
		if (src & ~src_d) q = 1'b1;   // CLK rising edge, D = +5 V
		if (irqack)       q = 1'b0;   // /CLR is asynchronous and wins
		irq_q <= q;
		src_d <= src;
	end
	assign irq_n = ~irq_q;            // the 6502 takes /Q (pin 8)

	// ---- 16C LS240 ---------------------------------------------------------
	// The inverting buffer, assembled per the bit map in the header.
	wire d7 = mame_polarity ?  test_n     : ~test_n;
	wire d6 = mame_polarity ? ~cmd_pending :  cmd_pending;
	assign rdio_data = { d7, d6, resp_pending, ~test_n,
	                     1'b0, 1'b0, coin1, coin2 };

endmodule
