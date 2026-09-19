`timescale 1ns/1ps
//============================================================================
//  Bad Lands ROM download router -- the MiSTer index-0 ioctl stream -> region
//  writes.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Ported from
//  `Arcade-Blasteroids_MiSTer/rtl/mem/blstroid_rom_loader.sv` (GPL-3.0,
//  same author; itself from Xybots and Toobin').  The stream map, the region
//  widths and the absence of an EEPROM region are Bad Lands' own.
//
//  The stream:
//    stream base   size       MAME region  destination
//    0x000000      0x040000   maincpu      68000 program ROM -> BRAM (word)
//    0x040000      0x010000   audiocpu     6502 sound ROM    -> BRAM
//    0x050000      0x060000   tiles        playfield stamps  -> SDRAM
//    0x0B0000      0x030000   sprites      MO stamps         -> SDRAM
//    total         0x0E0000                (917 504 bytes)
//
//  Four regions, no holes, no padding, no ROM_RELOAD anywhere, so the decode
//  below is a pure comparison chain.  `snd_rom_addr` counts from 0 at stream
//  0x040000, so it is the 6502's own ROM offset.
//
//  There is no EEPROM region: the MRA has no `<nvram>` element and the 2816A
//  is not part of the index-0 image -- the part ships erased and the game
//  writes its own defaults from the self test.  `badlands_nvram_io` takes
//  index 2 and index 2 only.
//
//  Byte order: `maincpu` is emitted as big-endian interleaved words, the MRA's
//  `map="01"` socket (20F / 17F) supplying the first byte of each output word
//  = D15:8.  So download byte 2N is the 68000 word's high lane and 2N+1 the
//  low lane, and `badlands_prog_rom` splits them exactly that way; getting it
//  backwards byte-swaps every opcode with no error message anywhere.  The two
//  graphics regions use the opposite, little-endian packing (even byte ->
//  D7:0, odd -> D15:8) because the stamp ROMs are byte-addressed devices with
//  no word order of their own; that packing lives in `badlands_sdram_loader`.
//
//  Back-pressure: the BRAM regions take a byte per clock and never stall.  The
//  two SDRAM regions go through `badlands_sdram_loader`, which packs byte
//  pairs into words and raises `wr_busy` while a word is outstanding;
//  `badlands_main` wires `ioctl_wait = sdl_wr_busy`.  This module deliberately
//  does not consume `ioctl_wait`: hps_io holds `ioctl_addr`/`ioctl_dout` and
//  simply does not issue the next `ioctl_wr`, so the decode is unaffected.
//
//  Outputs are registered, one clk_sys after the ioctl_wr.  `rom_loaded`
//  latches at the end of the index-0 download only, with a defined power-up
//  value, so an index-2 NVRAM restore on a core that has never seen a ROM
//  download cannot announce "ROMs are loaded".
//============================================================================

module badlands_rom_loader
(
	input  logic        clk,
	input  logic        reset,          // ~pll_locked only, never the game reset

	// ---- HPS ioctl download channel ----
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,     // byte offset into the index-0 stream
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,

	// ---- 68000 program ROM (BRAM), byte port ----
	output logic        prog_rom_wr,
	output logic [17:0] prog_rom_addr,  // byte offset 0..0x3FFFF
	output logic  [7:0] prog_rom_data,

	// ---- 6502 sound ROM (BRAM), byte port ----
	output logic        snd_rom_wr,
	output logic [15:0] snd_rom_addr,   // byte offset 0..0xFFFF = the 6502's own
	output logic  [7:0] snd_rom_data,

	// ---- playfield stamps -> SDRAM (via badlands_sdram_loader) ----
	output logic        tiles_wr,
	output logic [18:0] tiles_addr,     // byte offset 0..0x5FFFF

	// ---- motion-object stamps -> SDRAM (via badlands_sdram_loader) ----
	output logic        sprites_wr,
	output logic [17:0] sprites_addr,   // byte offset 0..0x2FFFF

	// shared byte for both SDRAM regions (they never strobe in the same cycle)
	output logic  [7:0] gfx_data,

	// ---- high once the index-0 download has ended ----
	output logic        rom_loaded
);

	// region bases in the index-0 stream (see the header table)
	localparam [26:0] BASE_PROG  = 27'h000000;
	localparam [26:0] BASE_SND   = 27'h040000;
	localparam [26:0] BASE_TILES = 27'h050000;
	localparam [26:0] BASE_SPR   = 27'h0B0000;
	localparam [26:0] END_STREAM = 27'h0E0000;

	wire is_idx0 = (ioctl_index == 16'd0);
	wire wr      = ioctl_download & ioctl_wr & is_idx0;

	wire in_prog  = wr && (ioctl_addr <  BASE_SND);        // BASE_PROG == 0
	wire in_snd   = wr && (ioctl_addr >= BASE_SND)   && (ioctl_addr < BASE_TILES);
	wire in_tiles = wr && (ioctl_addr >= BASE_TILES) && (ioctl_addr < BASE_SPR);
	wire in_spr   = wr && (ioctl_addr >= BASE_SPR)   && (ioctl_addr < END_STREAM);

	logic [7:0] data_r;

	always_ff @(posedge clk) begin
		data_r        <= ioctl_dout;
		prog_rom_wr   <= in_prog  & ~reset;
		snd_rom_wr    <= in_snd   & ~reset;
		tiles_wr      <= in_tiles & ~reset;
		sprites_wr    <= in_spr   & ~reset;
		prog_rom_addr <= 18'(ioctl_addr - BASE_PROG);
		snd_rom_addr  <= 16'(ioctl_addr - BASE_SND);
		tiles_addr    <= 19'(ioctl_addr - BASE_TILES);
		sprites_addr  <= 18'(ioctl_addr - BASE_SPR);
	end

	// one registered byte, fanned out to every destination port
	assign prog_rom_data = data_r;
	assign snd_rom_data  = data_r;
	assign gfx_data      = data_r;

	// rom_loaded latches on the falling edge of the index-0 download.  `reset`
	// (~pll_locked) gives it a defined value from power-up, so no consumer
	// ever samples an X.
	wire  dl0 = ioctl_download & is_idx0;
	logic dl0_d;
	always_ff @(posedge clk) begin
		if (reset) begin
			dl0_d      <= 1'b0;
			rom_loaded <= 1'b0;
		end else begin
			dl0_d <= dl0;
			if (dl0)        rom_loaded <= 1'b0;   // download in progress
			else if (dl0_d) rom_loaded <= 1'b1;   // first cycle after it ends
		end
	end

endmodule
