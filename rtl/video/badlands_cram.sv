`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the colour-RAM port: the 2 K x 8 SRAM at 8K, the 10K F374
//  address latch, the 8J A0 mux, the 19K/8F CPU path, the 3K byte-0 holding
//  register, the 1K/5K HC273 output registers and the 14E/4J PIXCLK gate.
//  Schematic sheet 4 grids 8K, 10K, 19K, 8F, 8J, 3K, 1K, 5K, 14E, 4J.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The SRAM is one 2 K x 8, 35 ns part (137534-001) with A10 (19) and A9 (22)
//  tied to GND, so only 512 of its bytes exist = 256 colours x 2 bytes.
//  /CS = GND, /WE = /CRAMWP, /OE = /CRAMOE, A8:A1 from the shared bus and
//  A0 from the 8J mux.  This module models those 512 bytes only.
//
//  Both address paths are inverted, which is why PRIO = /MO:
//      video: A8:A1 = 10K Q = {PRIO, /CRAMA7:1} = NOT {mo_selected, index}
//             A0    = 7M
//      CPU  : A8:A1 = NOT (68000 A9:A2) through the inverting 19K LS240
//             A0    = 68000 A1, which does NOT go through the inverter
//  The two inversions cancel at software level and leave true index bit 7 =
//  NOT PRIO: playfield colours 0x00-0x7F, motion objects 0x80-0xFF.
//
//  Two reads per pixel.  10K latches at 7M falling; then 7M = 0 gives A0 = 0
//  (byte 0 = colour word D15:8, held by 3K at the next 7M rising edge) and
//  7M = 1 gives A0 = 1 (byte 1 = D7:0, taken by 5K directly).  Both halves land
//  in 1K/5K at the next PIXCLK: one 16-bit colour per 7M period out of two SRAM
//  accesses at the 14M rate.
//
//      byte 0 (A1 = 0) = { GREEN0, RED5, RED4, RED3, RED2, RED1, GREEN5, GREEN4 }
//      byte 1 (A1 = 1) = { GREEN3, GREEN2, GREEN1, BLUE5, BLUE4, BLUE3, BLUE2, BLUE1 }
//
//  1K and 5K are 74HCT273s whose /CLR is /BLANK, so the colour word is forced
//  to 0 (black) throughout horizontal and vertical blanking.  That is the only
//  blanking mechanism on the board; sheet 6 has no /BLANK input.
//
//  PIXCLK:  14E F74 half 1: D = /CRAM, CK = 7M, /S = +5 V, /CLR = /CRAM (the
//                           same net, asynchronously)
//           4J F260       : PIXCLK = NOT( 14E./Q OR 7M )
//  With no colour-RAM access PIXCLK = NOT 7M, one clock per pixel.  While /CRAM
//  is low, and until the first 7M rising edge after it releases, PIXCLK is held
//  low and 1K/5K hold the previous pixel's colour: a /CRAM cycle repeats
//  exactly 3 pixels, because /AS is low for 3 of the 68000's 4 bus clocks.
//  Touching colour RAM during active display therefore shows on screen; MAME
//  has no equivalent artefact.  The same /CRAM gate tri-states 10K, switches
//  A0 from 7M to CPU A1 and enables 19K and 8F, so the whole port swaps to the
//  CPU atomically.
//
//  The 8K SRAM is asynchronous on the board, but here it is one write port and
//  two REGISTERED read ports, both on clk_sys — keep it that way.  A Cyclone V
//  M10K read is always registered and no block RAM in the family has three
//  ports, so a combinational model costs roughly 3 300 ALMs and 4 400 registers
//  for 512 bytes; this shape fits in 2 M10K (Quartus duplicates the array once
//  per read port).  A second WRITE port is what it refuses.  Registering is
//  invisible on both sides: the video reads are sampled at the 3K and 5K edges
//  and the SRAM address is constant for the four clk_sys before each of them,
//  and the CPU byte is still presented about 16 clk_sys before fx68k latches
//  D15:8 at the end of S6, so the 0-wait-state /CRAM cycle is untouched.
//============================================================================

module badlands_cram (
	input  logic       clk,
	input  logic       reset,

	input  logic       m7,          // 7M level: the 8J A0 mux and the SRAM read
	input  logic       ce_m7_rise,  // 7M rising  (ph 7): 3K, 14E
	input  logic       ce_m7_fall,  // 7M falling (ph 3): 10K, PIXCLK
	input  logic       blank_n,     // /BLANK -> 1K/5K /CLR

	// ---- video side -------------------------------------------------------
	input  logic [6:0] crama_n,     // the shared /CRAMA7:1 bus (already muxed)
	input  logic       prio,        // GAL 9N pin 17

	// ---- CPU side ---------------------------------------------------------
	input  logic       cram_cs,     // /CRAM asserted (level, follows /AS)
	input  logic [9:1] cram_a,      // 68000 A9:A1; A12:A10 are undecoded (x8 mirror)
	input  logic       cram_we,     // one clk_sys pulse: write the byte
	input  logic [7:0] cram_din,    // the 68000's D15:8
	output logic [7:0] cram_dout,   // -> the 68000's D15:8, registered

	// ---- to the DAC -------------------------------------------------------
	output logic [7:0] byte0,       // 1K HC273
	output logic [7:0] byte1,       // 5K HC273
	output logic       pixclk_rise, // observation point (one clk_sys pulse)
	// Not a PCB net: the true colour index (0..255) whose word is in
	// byte0/byte1, captured on the same PIXCLK edge so the pixel repeat above
	// leaves it unchanged.
	output logic [7:0] pix_index
);

	// 512 used bytes: A10 and A9 are grounded
	logic [7:0] mem [0:511];

	// =====================================================================
	// 1. 10K 74F374 — the video address latch (CK = /7M rising = 7M falling)
	// =====================================================================
	// The register keeps clocking during a CPU access; only its OUTPUT is
	// tri-stated (/OE = CRAM), which the address mux below models.
	logic [7:0] q10k;               // {PRIO, /CRAMA7:1} -> 8K A8:A1
	always_ff @(posedge clk) begin
		if (reset)        q10k <= 8'hFF;
		else if (ce_m7_fall) q10k <= {prio, crama_n};
	end

	// =====================================================================
	// 2. The address mux — the inverting 19K LS240 and the 8J A0 mux
	// =====================================================================
	wire [7:0] cpu_a8_1 = ~cram_a[9:2];          // 19K inverts; A1 does not go through
	wire       a0       = cram_cs ? cram_a[1]    // 8J: (CRAM AND A1)
	                              : m7;          //     OR (/CRAM AND 7M)
	wire [8:0] phys     = cram_cs ? {cpu_a8_1, a0} : {q10k, a0};
	wire [8:0] cpu_phys = {cpu_a8_1, cram_a[1]};

	// ---- the array: one write port, two registered read ports -------------
	// `cram_we` is /CRAMWP, which pulses low on a CPU upper-byte write.
	//   read 1 -> `cram_dout`, the 8F LS245 path to the 68000
	//   read 2 -> `sram_q`,    the shared 8K data bus
	// See the header for why registering both is invisible on both sides.
	logic [7:0] cpu_q, vid_q;
	always_ff @(posedge clk) begin
		if (cram_we) mem[cpu_phys] <= cram_din;
		cpu_q <= mem[cpu_phys];
		vid_q <= mem[phys];
	end

	assign cram_dout  = cpu_q;
	wire [7:0] sram_q = vid_q;

	// =====================================================================
	// 3. 14E F74 + 4J F260 — PIXCLK
	// =====================================================================
	logic q14e;
	always_ff @(posedge clk) begin
		if (reset)        q14e <= 1'b1;
		else if (cram_cs) q14e <= 1'b0;   // /CLR = /CRAM, asynchronous
		else if (ce_m7_rise) q14e <= 1'b1;   // D = /CRAM = 1
	end
	// PIXCLK = NOT(14E./Q OR 7M) = q14e AND NOT 7M, so it rises on a 7M falling
	// edge and only while 14E has been released.
	assign pixclk_rise = ce_m7_fall & q14e & ~cram_cs;

	// =====================================================================
	// 4. 3K LS374 (7M rising) and 1K / 5K HC273 (PIXCLK, /CLR = /BLANK)
	// =====================================================================
	logic [7:0] q3k;
	always_ff @(posedge clk) begin
		if (reset)        q3k <= 8'h00;
		else if (ce_m7_rise) q3k <= sram_q; // byte 0 read over the 7M = 0 half
	end

	always_ff @(posedge clk) begin
		if (reset || !blank_n) begin       // /CLR = /BLANK, asynchronous
			byte0 <= 8'h00;
			byte1 <= 8'h00;
		end else if (pixclk_rise) begin
			byte0 <= q3k;                  // 1K: the held byte 0
			byte1 <= sram_q;               // 5K: byte 1, straight off 8K D7:0
		end
	end

	// The SRAM address is the complement of the index on both ports, so the
	// index the program would have written is just ~10K's contents.
	always_ff @(posedge clk) begin
		if (reset)             pix_index <= 8'h00;
		else if (pixclk_rise)  pix_index <= ~q10k;
	end

endmodule
