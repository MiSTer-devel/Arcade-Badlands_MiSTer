`timescale 1ns/1ps
//============================================================================
//  Bad Lands 68000 program ROM -- 256 KB (131072 x 16) in on-chip block RAM.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Ported from
//  `Arcade-Blasteroids_MiSTer/rtl/mem/blstroid_prog_rom.sv` (GPL-3.0,
//  same author; itself from Xybots and Toobin').  Same size, same shape; the
//  socket map and the SLAPSTIC note are Bad Lands' own.
//
//  The four sockets:
//    20F 136074-1008  /OE = A17   D15:8   0x000000-0x01FFFF even bytes
//    27F 136074-1006  /OE = A17   D7:0    0x000000-0x01FFFF odd  bytes
//    17F 136074-1009  /OE = /A17  D15:8   0x020000-0x03FFFF even
//    24F 136074-1007  /OE = /A17  D7:0    0x020000-0x03FFFF odd
//  /CE = A23 on all four; ROM address = {A16,A15,A14S,A13S,A12..A1}.  The four
//  parts arrive as one flat 0x40000-byte `maincpu` image in the download
//  stream, so this module is one linear 128K-word array, the socket decode
//  disappears, and `rd_a` is simply the 68000's A17..A1.
//
//  SLAPSTIC socket 28C is empty and GAL 26C passes A13/A14 straight through
//  (`o18 = i3`, `o19 = i4`), so 0x008000-0x00FFFF is plain ROM with nothing to
//  bank.  A18-A22 are undecoded on the board, so the whole A23 = 0 half
//  mirrors every 0x40000; that mirroring belongs to badlands_addr_decode.
//
//  Byte order: inside the MRA's `<interleave output="16">`, `map="01"`
//  supplies the first byte of each output word (big-endian D15:8) and
//  `map="10"` the second (D7:0).  So download byte 2N is the 68000 word's high
//  lane and 2N+1 the low lane.  Reversing it byte-swaps every opcode; the CPU
//  then fetches a garbage reset vector and runs nonsense from its first
//  instruction, with no error message anywhere.
//
//  Read port: `rd_a` is the 68000 word address A17..A1.  Registered read --
//  data is valid one clk after the address, and the address must be stable for
//  that clock, which a 68000 bus cycle guarantees many times over.
//
//  Resource: 128K x 8 per lane = 128 M10K per lane, 256 M10K total, 46 % of
//  the DE10-Nano's 553 blocks in this one module.  Two byte-wide arrays rather
//  than one 16-bit array with bit-select writes: it is the textbook
//  byte-enable inference pattern and removes any doubt about Quartus picking
//  M10K over LABs.  No megafunction, no init file -- the loader fills it at
//  download.
//============================================================================

module badlands_prog_rom
(
	input  logic        clk,

	// ---- loader write port (byte stream from badlands_rom_loader) ----
	input  logic        wr,
	input  logic [17:0] wr_addr,    // byte offset 0..0x3FFFF
	input  logic  [7:0] wr_data,

	// ---- 68000 read port (16-bit big-endian word) ----
	input  logic [16:0] rd_a,       // 68000 A17..A1
	output logic [15:0] rd_data
);

	localparam int WORDS = 131072;   // 0x40000 bytes / 2

	logic [7:0] mem_hi [0:WORDS-1];   // even download byte -> D15:8
	logic [7:0] mem_lo [0:WORDS-1];   // odd  download byte -> D7:0

	// Power-up to 0.  Quartus zeroes M10K contents on configuration, so this
	// loop is simulation only -- it keeps iverilog and Verilator free of X on
	// an unwritten hole -- and is skipped during synthesis, where 131072
	// iterations exceed the unroll limit.
`ifndef ALTERA_RESERVED_QIS
	initial begin
		for (int i = 0; i < WORDS; i++) begin
			mem_hi[i] = '0;
			mem_lo[i] = '0;
		end
	end
`endif

	logic [7:0] q_hi, q_lo;

	always_ff @(posedge clk) begin
		if (wr && !wr_addr[0]) mem_hi[wr_addr[17:1]] <= wr_data;
		if (wr &&  wr_addr[0]) mem_lo[wr_addr[17:1]] <= wr_data;
		q_hi <= mem_hi[rd_a];
		q_lo <= mem_lo[rd_a];
	end

	assign rd_data = {q_hi, q_lo};

endmodule
