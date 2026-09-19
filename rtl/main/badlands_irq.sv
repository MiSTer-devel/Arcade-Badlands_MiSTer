`timescale 1ns/1ps
//============================================================================
//  Bad Lands interrupts -- the 22J 74F74 VBLANK latch and the 13J 74F00 that
//  encodes /IPL0, from schematic sheet 2.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Structure ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_irq.sv` (GPL-3.0, same
//  author).
//
//  Wiring:
//    22J F74 half 2:  D = +5 V, CLK = VBLANK (rising), /CLR = /VIRQACK (level),
//                     Q = VIRQ
//    13J F00 (NAND):  /IPL0 = NOT( /AUDIRQ AND VIRQ )
//                     /IPL1 = /AUDIRQ      /IPL2 = +5 V
//
//   | /AUDIRQ | VIRQ | level | meaning                                    |
//   |    1    |   0  |   0   | no interrupt                               |
//   |    1    |   1  |   1   | IRQ1 = VBLANK                              |
//   |    0    |   x  |   2   | IRQ2 = sound mailbox; a pending VBLANK is  |
//   |         |      |       | hidden, not lost, and returns as level 1   |
//
//  /IPL2 is tied high, so the board never presents level 3 or above.
//
//  Acknowledges:
//  * IRQ1 is cleared by a write to 0xFE2001 (/VIRQACK).  /CLR dominates the
//    clock, so a VBLANK edge during the acknowledge does not set the latch.
//  * IRQ2 is cleared when the 68000 reads 0xFEA000 (/AUDRD); that flop lives
//    in the sound mailbox and only its level arrives here.
//  * Every interrupt is autovectored (/VPA): level 1 -> vector 25, level 2 ->
//    vector 26.
//
//  Reset: /RES does not reach this half of 22J on the board, so `reset` is a
//  power-up initialiser only.  A watchdog reset leaves a pending VBLANK
//  request pending, as the hardware does.
//============================================================================

module badlands_irq
(
	input  logic       clk,        // clk_sys
	input  logic       reset,      // power-up only: sets VIRQ = 0

	input  logic       vblank,     // SOS-2 pin 22, active high
	input  logic       virqack_n,  // LS138 24C Y1, level, active low (write 0xFE2001)
	input  logic       audirq_n,   // 15J sheet 5, active low -> /IPL1 directly

	output logic [2:0] ipl,        // {/IPL2, /IPL1, /IPL0}, active low
	output logic       virq        // 22J Q
);

	// ---------------- 22J 74F74 half 2 -----------------------------------
	// Set by the rising edge of VBLANK, cleared while /VIRQACK is low.
	// /CLR dominates the clock on a 74F74, so the clear is tested first.
	logic vblank_q;
	always_ff @(posedge clk) vblank_q <= vblank;
	wire vblank_rise = vblank & ~vblank_q;

	always_ff @(posedge clk) begin
		if (reset)            virq <= 1'b0;   // power-up only
		else if (!virqack_n)  virq <= 1'b0;   // /CLR, level sensitive
		else if (vblank_rise) virq <= 1'b1;   // D = +5 V, CLK = VBLANK rising
	end

	// ---------------- 13J 74F00 pin 8 and the two direct nets -------------
	assign ipl[0] = ~( audirq_n & virq );   // /IPL0 = NAND(/AUDIRQ, VIRQ)
	assign ipl[1] = audirq_n;               // /IPL1 = /AUDIRQ, direct
	assign ipl[2] = 1'b1;                   // /IPL2 = +5 V

endmodule
