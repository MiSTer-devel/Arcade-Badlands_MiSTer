`timescale 1ns/1ps
//============================================================================
//  MiSTer transport / save-request controller for the Bad Lands 2816A.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_nvram_io.sv` (GPL-3.0,
//  same author; itself from Xybots and Toobin'), re-based on this core's
//  download stream.
//
//  ---- one channel, not two ----------------------------------------------
//  Blasteroids appends its 512-byte EEPROM image to the end of the index-0 MRA
//  stream and therefore has to accept loader bytes on two channels.  Bad Lands
//  does not: its index-0 stream is exactly 0x0E0000 bytes of maincpu, audiocpu,
//  tiles and sprites, and the EEPROM is not part of it -- the part ships erased
//  and the game writes its own defaults from the self test.
//
//  So the only loader path here is `ioctl_index == 2`, the MRA's
//  `<nvram index="2" size="2048"/>` restore/save channel, which is also what
//  adds the OSD "Save Settings" entry; a byte at any index-0 address is
//  ignored.  Adding an index-0 case would silently corrupt the EEPROM with
//  graphics data, because the index-0 stream is longer than 2 KB and its low
//  addresses are program ROM.
//
//  All coin and game options live in this EEPROM -- there are no option DIP
//  switches (manual page 1-9; SW1 is four development options that must stay
//  off).  Losing it resets the coin options to the green defaults and
//  increments the self-test error count.
//
//  ---- the save request ---------------------------------------------------
//  A programmed byte sets `dirty`; after SETTLE_CYCLES of quiet, one upload is
//  requested.  The settle is what turns a self-test page's burst of option
//  writes into a single save.
//
//  Dirty state deliberately survives a board or watchdog reset -- the physical
//  EEPROM does, so dropping the flag there could lose a freshly programmed
//  option or high score.  Only FPGA initialisation or an NVRAM load
//  establishes a clean image.  If a write and an upload start coincide, the
//  write wins and is requested again.
//============================================================================

module badlands_nvram_io #(
	// ~1.17 s at 57.272724 MHz: long enough that a burst of option writes from
	// one self-test page produces a single upload.
	parameter int unsigned SETTLE_CYCLES = 67_108_863
)(
	input  logic        clk,
	input  logic        init_reset,

	input  logic        write_accepted,   // from badlands_eeprom_2816

	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,
	input  logic        ioctl_upload,

	output logic        load_we,
	output logic [10:0] load_addr,
	output logic  [7:0] load_data,
	output logic [10:0] dump_addr,
	input  logic  [7:0] dump_data,

	output logic        ioctl_upload_req,
	output logic  [7:0] ioctl_upload_index,
	output logic  [7:0] ioctl_din
);

	// Index 2 only, see the header.  The 2 KB part takes ioctl_addr[10:0], so
	// an oversized `.nvm` file simply wraps.
	wire index2_nvram = (ioctl_index == 16'd2);

	assign load_we   = ioctl_download & ioctl_wr & index2_nvram;
	assign load_addr = ioctl_addr[10:0];
	assign load_data = ioctl_dout;
	assign dump_addr = ioctl_addr[10:0];

	localparam int SETTLE_W = (SETTLE_CYCLES <= 1) ? 1 : $clog2(SETTLE_CYCLES + 1);
	logic [SETTLE_W-1:0] settle;
	logic dirty;
	wire  settled      = (settle == SETTLE_W'(SETTLE_CYCLES));
	wire  upload_start = ioctl_upload & index2_nvram;

	always_ff @(posedge clk) begin
		if (init_reset) begin
			dirty  <= 1'b0;
			settle <= '0;
		end else if (write_accepted) begin
			dirty  <= 1'b1;      // a new byte cannot be cleaned by a concurrent upload
			settle <= '0;
		end else if (load_we || upload_start) begin
			dirty  <= 1'b0;
			settle <= '0;
		end else if (dirty && !settled) begin
			settle <= settle + 1'b1;
		end
	end

	assign ioctl_upload_req   = dirty & settled;
	assign ioctl_upload_index = 8'd2;
	assign ioctl_din          = dump_data;

	/* verilator lint_off UNUSEDSIGNAL */
	// ioctl_addr[26:11] selects nothing: the part is 2 KB and the index-2 file
	// is 2 KB.  Named so the lint stays clean and the fact stays visible.
	wire _unused_hi = &{1'b0, ioctl_addr[26:11]};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
