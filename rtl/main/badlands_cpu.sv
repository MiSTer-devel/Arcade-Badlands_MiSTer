`timescale 1ns/1ps
//============================================================================
//  Bad Lands main CPU -- fx68k (cycle-exact 68000) wired as schematic sheet 2
//  draws the 22E MC68000.  This is the only 68000 in the core.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  The CPU itself is `core/rtl/lib/fx68k/` -- Jorge
//  Cwik's fx68k, GPL-3.0-or-later, vendored verbatim from the Xybots core.
//  Derived from `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_cpu.sv`
//  and `Arcade-Xybots_MiSTer/rtl/main/xybots_cpu.sv` (GPL-3.0, same
//  author), which clock a 68000 at crystal/2 the same way.
//
//  ---- the clock phase, and why it matters --------------------------------
//  The SOS-2's 7M pin drives one load: 23R 74F74 half 2's D input, clocked by
//  14M rising.  Q is the board's `7M` and /Q is `/7M`:
//
//      7M  -> GAL 26R CLK, GAL 21R input, GAL 9N, LB ACLK, the H counter
//      /7M -> 68000 22E CLK (pin 15), 28R LS163A CLK (pin 2), 7N, 10K
//
//  The CPU and the /DTACK sequencer run on /7M while GAL 26R and the video
//  side run on 7M; that half-cycle stagger is deliberate.
//
//  Here clk_sys = 57.272724 MHz, so one `7M` period is 8 clk_sys and pcnt[2:0]
//  is the position inside it, pcnt == 0 being the `ce_7m` cycle = the `7M`
//  rising edge.  An enable is high in the clk_sys cycle whose ending edge is
//  the modelled edge, so `7M` is high over pcnt 1..4 and /7M rises at the edge
//  ending pcnt == 4:
//
//      enPhi1 = (pcnt == 4)      // /7M rising -- the CPU clock's high phase
//      enPhi2 = (pcnt == 0)      // 4 clk_sys later, /7M falling = ce_7m
//
//  The two must stay 4 clk_sys apart: fx68k drives one half of every CPU clock
//  from each, so an uneven split silently moves where /DTACK is sampled
//  relative to badlands_vram's /RTIME window.  Blasteroids' wrapper has
//  enPhi1 = ce_7m because its 68000 is not staggered; ours is.  badlands_dtack
//  takes the same (pcnt == 4) tick, which is what sheet 2 wires to its LS163A.
//
//  ---- pins as sheet 2 wires them -----------------------------------------
//    15 CLK        = /7M                        -> enPhi1/enPhi2 above
//    17 HALT, 18 RESET = /RES through R58/R59 1 K SERIES resistors -> `reset`
//    10 DTACK      = 19J LS08 pin 3   (`dtack` here is active high)
//    21 VPA        = 24J F20 pin 6    (`vpa_n`, active low)
//    23 IPL2       = +5 V -- the board never presents a level above 2
//    24 IPL1       = /AUDIRQ ; 25 IPL0 = 13J F00 pin 8   (badlands_irq)
//    12,13,22 BGACK,BR,BERR = one R66 1 K pull-up: no bus arbitration and no
//                    bus-error timeout anywhere on this board
//    11,19,20 BG,VMA,E = unconnected
//    26,27,28 FC2,FC1,FC0 -> 24J F20 (the /VPA NAND)
//    29-52 A1-A23  = a[23:1].  There is no A0 on the board; the byte lane is
//                    /UDS //LDS and nothing else.
//
//  /VPA = NOT( AS & FC2 & FC1 & FC0 ).  FC = 7 is CPU space, which the 68000
//  drives only during an interrupt acknowledge, so every interrupt is
//  autovectored: level 1 -> vector 25 @ 0x64, level 2 -> vector 26 @ 0x68.
//  The same net drives the 28R LS163A's ENT, so an IACK gets no /DTACK and can
//  only end through the CPU's own VPA path -- badlands_dtack's business, not
//  this module's.  Here we just publish `fc` and take `vpa_n`.
//
//  ---- three deviations from a literal transcription ----------------------
//  1. HALTn is tied inactive rather than to the /RES net.  fx68k's `extReset`
//     performs the whole 68000 reset, and its author documents HALTn as single
//     step only, to be forced high if unused; internally it only gates
//     busAvail in the bus arbiter, and this board has no second master.
//  2. `pwrUp` is driven from the same `reset`, as fx68k's own top level does;
//     a watchdog timeout is a full CPU reset either way.
//  3. BERRn is tied inactive: pins 12/13/22 share one 1 K pull-up and there is
//     no bus-error source at all.
//
//  `reset_inst` is high while a `RESET` instruction runs, which on this board
//  is observable nowhere: R58/R59 are series resistors, so pin 18 driven low
//  through 1 K cannot pull the /RES net (a totem-pole LS197 QD output) down.
//  It does not re-lock the EEPROM and does not reset the sound section --
//  unlike Blasteroids, where /HALT and /RESET tie directly to the net.  The
//  port exists so the fact is visible; badlands_main leaves it unconnected.
//============================================================================

module badlands_cpu
(
	input  logic        clk,        // clk_sys, 57.272724 MHz
	input  logic        ce_7m,      // clk_sys/8, the `7M` RISING edge (pcnt == 0)
	input  logic        reset,      // active high: the /RES net + power-up

	input  logic  [2:0] ipl,        // active low, "111" = none (badlands_irq)

	output logic [23:1] a,
	output logic [15:0] wdata,
	input  logic [15:0] rdata,
	output logic        as,         // active-high copy of /AS (15K F04 3->4)
	output logic        uds_n,
	output logic        lds_n,
	output logic        rw,         // 1 = read, 0 = write
	input  logic        dtack,      // active high (= ~/DTACK)
	input  logic        vpa_n,      // 24J F20 pin 6 -> 22E pin 21, active low
	output logic  [2:0] fc,
	output logic        reset_inst, // 1 while a RESET instruction runs (see header)

	// the /7M rising tick, so badlands_dtack's LS163A can use the very same
	// edge sheet 2 wires to its pin 2
	output logic        ce_7m_n
);

	// ------------------------------------------------------- clock phases
	// ce_7m loads pcnt with 1, so pcnt == 0 falls in the ce_7m cycle and runs
	// 1..7 through the rest of the period.
	logic [2:0] pcnt;
	always_ff @(posedge clk) begin
		if (ce_7m) pcnt <= 3'd1;
		else       pcnt <= pcnt + 3'd1;
	end

	wire en_phi1 = (pcnt == 3'd4);   // /7M rising
	wire en_phi2 = ce_7m;            // /7M falling = pcnt 0, exactly 4 clk later
	assign ce_7m_n = en_phi1;

	// ------------------------------------------------------------------ IPL
	// fx68k samples IPL under enPhi2 into its own two-deep synchroniser, so
	// ordinary posedge staging here is correct; the extra latency is invisible
	// at the /7M sampling rate.  Reset value 3'b111 = no interrupt.
	logic [2:0] ipl_r, ipl_rr;
	always_ff @(posedge clk) begin
		if (reset) begin
			ipl_r  <= 3'b111;
			ipl_rr <= 3'b111;
		end else begin
			ipl_r  <= ipl;
			ipl_rr <= ipl_r;
		end
	end

	// ---------------------------------------------------------------- fx68k
	wire        fx_as_n, fx_reset_n;
	wire [23:1] fx_eab;

	/* verilator lint_off UNUSEDSIGNAL */
	wire        fx_e, fx_vma_n;    // 6800 bus: 22E pins 19, 20 unconnected
	wire        fx_bg_n;           // 22E pin 11 unconnected, no second master
	wire        fx_halted_n;       // double bus fault; no load on sheet 2
	/* verilator lint_on UNUSEDSIGNAL */

	assign a          = fx_eab;
	assign as         = ~fx_as_n;
	assign reset_inst = ~fx_reset_n;

	fx68k u_fx68k (
		.clk      (clk),
		.HALTn    (1'b1),          // deviation 1
		.extReset (reset),
		.pwrUp    (reset),         // deviation 2
		.enPhi1   (en_phi1),
		.enPhi2   (en_phi2),

		.eRWn     (rw),            // 1 = read, 0 = write
		.ASn      (fx_as_n),
		.LDSn     (lds_n),
		.UDSn     (uds_n),
		.E        (fx_e),
		.VMAn     (fx_vma_n),

		.FC0      (fc[0]),
		.FC1      (fc[1]),
		.FC2      (fc[2]),
		.BGn      (fx_bg_n),
		.oRESETn  (fx_reset_n),
		.oHALTEDn (fx_halted_n),

		.DTACKn   (~dtack),
		.VPAn     (vpa_n),
		.BERRn    (1'b1),          // deviation 3
		.BRn      (1'b1),
		.BGACKn   (1'b1),
		.IPL0n    (ipl_rr[0]),
		.IPL1n    (ipl_rr[1]),
		.IPL2n    (ipl_rr[2]),

		.iEdb     (rdata),
		.oEdb     (wdata),
		.eab      (fx_eab)
	);

endmodule
