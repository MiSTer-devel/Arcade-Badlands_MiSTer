`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- the 6502 sound bus: 6502A 14C (T65) plus
//  the two memories it addresses, 6264 9B (8 KB work RAM) and 27512 9C (64 KB
//  program ROM with the $3000-$3FFF bank window).
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  The T65 instantiation pattern, the registered-read memory structure and the
//  "read mux gated by the strobe that enables each chip's own output driver"
//  convention come from the Blasteroids MiSTer core's `blstroid_jsa.sv` /
//  `blstroid_jsa_bus.sv` / `blstroid_jsa_mem.sv` (GPL-3.0, same author).
//  Everything device-specific is Bad Lands' own; this is not a JSA board.
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  6502A 14C, schematic sheet 5:
//     pin 37  Ø0 in   = 2H = 1.789772 MHz          -> `ce_2h`
//     pin 39  Ø2 out  = PH2 -> GAL 12E pin 2       -> `ce_2h` (see below)
//     pin 34  R/W     = SR/W (1 = read)
//     pin  2  RDY     = +5 V -- the sound CPU never waits
//     pin 38  SO      = +5 V   (falling-edge sensitive, so never active)
//     pin  6  /NMI    = /MAINIRQ  (15J pin 6)
//     pin  4  /IRQ    = 14E F74 pin 8 (/Q)
//     pin 40  /RES    = /AUDRES
//     pin  3  Ø1 out, pin 7 SYNC : stubs
//  Vectors in 136074-1018.9c: RESET $4000, NMI $57A0, IRQ/BRK $41BD.
//  `Rdy` is tied to 1 because the schematic ties RDY to +5 V.  There is no
//  DTACK-like mechanism on this bus: every 6502 cycle is exactly one 2H period.
//  Do not add wait states.
//
//  PH2: one `ce_2h` clock enable IS one 6502 bus cycle, so `ce_2h` plays PH2.
//  The address and (on a write) the data are stable for the whole clk_sys
//  period between two enables and the transfer happens on the enable itself.
//  The memories are read a clock ahead of the enable (registered BRAM output),
//  so their data is standing at the enable edge; the I/O strobes are PH2-gated
//  and true only on the enable, which is when T65 samples DI.
//
//  The memories:
//   * 6264 9B (8 K x 8, 120 ns, 137535-005): /CS1 = /SRAM, /WE = /SWR,
//     /OE = /SRD, A12..A0 = SA12..SA0.  $0000-$1FFF, no mirroring.
//   * 27512 9C (136074-1018.9c): /CE = /ROM, /OE = /SRD, A15,A14 = SA15,SA14,
//     A13 = SA13B, A12 = SA12B, A11..A0 = SA11..SA0.  $4000-$FFFF is the ROM
//     straight through; $3000-$3FFF is a 4 KB window over ROM $0000-$3FFF
//     selected by {BA13, BA12} from the 10E latch, bank n -> ROM offset
//     n * $1000.  ROM $0000-$2FFF is otherwise unreachable, and no instruction
//     ever executes from the window -- it is a data window.
//  /ROM is not PH2-qualified, so `rom_ce` alone would let the 27512 drive the
//  bus for the whole cycle; only its /OE (/SRD) carries PH2, which is why the
//  read mux below gates the ROM with `rom_ce & srd`.
//
//  Nothing drives SD7:0 on a read that selects no device, and sheet 5 shows no
//  pull-ups, so instead of inventing 0xFF for an open bus (which is what MAME
//  returns) the read mux holds the last byte a real driver put there -- what a
//  floating TTL bus does for the microseconds that matter.  The firmware
//  discards the byte from both of its /IRQACK reads ($41B9, $41C7) anyway.
//
//  The sound ROM is loaded, not initialised: `rom_wr` / `rom_wr_addr` /
//  `rom_wr_data` are the loader's write port into the 64 KB BRAM (download
//  offset 0x040000).  The array is a true dual-port M10K, loader on the write
//  side and the 6502 on the read side.  Do not fold the RAM and the ROM into a
//  shared output register: an M10K's output register cannot be shared, and
//  Quartus falls back to flip-flops (error 276003).
//============================================================================

module badlands_snd_bus
(
	input  logic        clk,
	input  logic        ce_2h,        // 6502 Ø0 = 2H = 1.789772 MHz; also PH2

	// ---- 6502 control lines ----
	input  logic        audres_n,     // /AUDRES -> 14C pin 40
	input  logic        irq_n,        // 14E /Q  -> 14C pin 4
	input  logic        nmi_n,        // /MAINIRQ -> 14C pin 6

	// ---- 10E LS174 bank bits ----
	input  logic  [1:0] bank,         // {BA13, BA12}

	// ---- I/O read sources ----
	input  logic  [7:0] rdp_data,     // 15F Q7:0  (main -> sound command latch)
	input  logic  [7:0] rdio_data,    // 16C LS240 ($2804)

	// ---- YM2151 9D ----
	output logic        ym_cs,        // /YAM asserted (address decode only)
	output logic        ym_a0,        // SA0
	output logic        ym_we,        // /SWR & /YAM
	output logic        ym_rd,        // /SRD & /YAM
	output logic  [7:0] ym_dout,      // 6502 -> YM
	input  logic  [7:0] ym_din,       // YM status -> 6502

	// ---- LS138 10F strobes (PH2-qualified, no direction term) ----
	output logic        sel_rdp,      // Y1 $2802
	output logic        sel_rdio,     // Y2 $2804
	output logic        sel_irqack,   // Y3 $2806  -- read OR write
	output logic        sel_wrp,      // Y5 $2A02
	output logic        sel_wrio,     // Y6 $2A04
	output logic        sel_y0,       // Y0 $2800  unconnected stub
	output logic        sel_y4,       // Y4 $2A00  unconnected stub
	output logic        sel_y7,       // Y7 $2A06  unconnected stub

	// ---- sound ROM load port ----
	input  logic        rom_wr,
	input  logic [15:0] rom_wr_addr,
	input  logic  [7:0] rom_wr_data,

	// ---- bus observability ----
	output logic [15:0] cpu_addr,
	output logic  [7:0] cpu_dout,
	output logic  [7:0] cpu_din,
	output logic        cpu_rnw,
	output logic        cpu_sync
);

	// ---- 6502A 14C ---------------------------------------------------------
	// Mode 00 = NMOS 6502 (the 14C part).  Enable = ce_2h, Rdy = 1.
	wire [7:0] dbgA_nc, dbgX_nc, dbgY_nc, dbgS_nc, dbgP_nc;
	T65_wrap u_cpu (
		.Mode(2'b00), .Res_n(audres_n), .Clk(clk), .Enable(ce_2h), .Rdy(1'b1),
		.IRQ_n(irq_n), .NMI_n(nmi_n), .DI(cpu_din),
		.A(cpu_addr), .DO(cpu_dout), .R_W_n(cpu_rnw), .Sync(cpu_sync),
		.dbg_A(dbgA_nc), .dbg_X(dbgX_nc), .dbg_Y(dbgY_nc),
		.dbg_S(dbgS_nc), .dbg_P(dbgP_nc) );

	// ---- GAL 12E + LS138 10F ------------------------------------------------
	wire rom_ce, yam, rest, sram, sa13b, sa12b, swr, srd, srd_gal;
	wire sel_ram, sel_ym, sel_bank, sel_romfix, sel_hole;
	wire [15:0] rom_rd_addr;
	badlands_snd_decode u_dec (
		.sa(cpu_addr), .srw(cpu_rnw), .ph2(ce_2h),
		.ba13(bank[1]), .ba12(bank[0]),
		.rom_ce(rom_ce), .yam(yam), .rest(rest), .sram(sram),
		.sa13b(sa13b), .sa12b(sa12b), .swr(swr), .srd(srd), .srd_gal(srd_gal),
		.sel_ram(sel_ram), .sel_ym(sel_ym), .sel_bank(sel_bank),
		.sel_romfix(sel_romfix), .sel_hole(sel_hole),
		.sel_y0(sel_y0), .sel_rdp(sel_rdp), .sel_rdio(sel_rdio),
		.sel_irqack(sel_irqack), .sel_y4(sel_y4), .sel_wrp(sel_wrp),
		.sel_wrio(sel_wrio), .sel_y7(sel_y7),
		.ym_a0(ym_a0), .rom_addr(rom_rd_addr) );

	// ---- 6264 9B : 8 KB work RAM -------------------------------------------
	(* ramstyle = "no_rw_check, M10K" *) logic [7:0] ram [0:8191];
	// ---- 27512 9C : 64 KB program ROM, filled by the ROM loader -------------
	(* ramstyle = "no_rw_check, M10K" *) logic [7:0] rom [0:65535];
`ifndef ALTERA_RESERVED_QIS
	// Sim-only zero fill so iverilog/verilator never read X out of an unwritten
	// location.  Quartus zeroes M10K contents at configuration anyway.
	initial begin
		for (int i = 0; i < 8192;  i++) ram[i] = 8'h00;
		for (int i = 0; i < 65536; i++) rom[i] = 8'h00;
	end
`endif

	// Both arrays are read every clk from the (stable) 6502 address, so their
	// registered outputs are standing at the ce_2h edge on which T65 samples DI.
	logic [7:0] ram_q, rom_q;
	always_ff @(posedge clk) begin
		if (swr & sel_ram) ram[cpu_addr[12:0]] <= cpu_dout;
		ram_q <= ram[cpu_addr[12:0]];
		rom_q <= rom[rom_rd_addr];
		if (rom_wr) rom[rom_wr_addr] <= rom_wr_data;
	end

	// ---- YM2151 9D strobes: /CS = /YAM, A0 = SA0, /WR = /SWR, /RD = /SRD ----
	assign ym_cs   = sel_ym;
	assign ym_we   = sel_ym & swr;
	assign ym_rd   = sel_ym & srd;
	assign ym_dout = cpu_dout;

	// ---- the SD7:0 read mux -------------------------------------------------
	// Each source is qualified by the strobe that actually turns on its output
	// driver on the board: 9B /OE = /SRD & /CS1 = /SRAM; 9C /OE = /SRD and
	// /CE = /ROM (PH2 comes in through /SRD, never through /ROM);
	// YM2151 /RD = /SRD & /CS = /YAM; 15F /OE = /RDP; 16C /1G,/2G = /RDIO.
	logic [7:0] sd_drv;
	logic       sd_hit;
	always_comb begin
		sd_hit = 1'b1;
		if      (srd & sel_ram)   sd_drv = ram_q;
		else if (srd & rom_ce)    sd_drv = rom_q;
		else if (srd & sel_ym)    sd_drv = ym_din;
		else if (sel_rdp)         sd_drv = rdp_data;
		else if (sel_rdio)        sd_drv = rdio_data;
		else begin sd_drv = 8'h00; sd_hit = 1'b0; end
	end

	// Bus hold: nobody drives SD7:0 on an /IRQACK read, inside the $2C00-$2FFF
	// hole or on a read of a dead LS138 output.  Remember the last byte a real
	// driver put on the bus instead of inventing 0xFF.
	logic [7:0] sd_last;
	initial sd_last = 8'h00;
	// `always @(posedge clk)` and not `always_ff`: this register has no reset,
	// so its power-up value comes from the `initial` above -- and IEEE 1800
	// 9.2.2.4 forbids an always_ff variable being written by another process.
	always @(posedge clk) if (ce_2h) begin
		if      (~cpu_rnw) sd_last <= cpu_dout;   // the 6502 drives on a write
		else if (sd_hit)   sd_last <= sd_drv;
	end

	assign cpu_din = sd_hit ? sd_drv : sd_last;

	// Nets that are inert here: the literal /SRD dump, the raw GAL selects and
	// the 27512's own A13/A12 pins, the T65 debug registers, and the
	// $2C00-$2FFF hole, which decodes nothing at all.
	wire _unused_bus = &{ 1'b0, srd_gal, rest, sram, yam, sa13b, sa12b,
	                      sel_bank, sel_romfix, sel_hole,
	                      dbgA_nc, dbgX_nc, dbgY_nc, dbgS_nc, dbgP_nc };

endmodule
