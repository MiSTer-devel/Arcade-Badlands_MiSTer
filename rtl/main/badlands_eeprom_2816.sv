`timescale 1ns/1ps
//============================================================================
//  Xicor 2816A (2 K x 8) EEPROM at 27J plus the 22J F74 unlock flop and the
//  19J LS08 that gates its /OE -- schematic sheet 2.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_eeprom_2804.sv`
//  (GPL-3.0, same author; itself from Xybots and Toobin').  Bad Lands fits a
//  full 2 K x 8 2816A, not Blasteroids' 512-byte X2804A, and its relock strobe
//  is /WRL -- every 68000 write anywhere in the map -- not a lane-qualified
//  one, so the addressing and the relock rule differ.
//
//  ---- wiring -------------------------------------------------------------
//    19,22,23,1  A10..A7 = 68000 A11, A10, A9, A8
//    2-8         A6..A0  = 68000 A7, A6, A5, A4, A3, A2, A1
//    17..9       D7..D0  = the 68000's D7:0 -- the odd / low byte lane
//    21 /WE = /WRL = R/W + /AS (17J F32) -- every 68000 write, unqualified
//    20 /OE = 19J LS08 pin 6
//    18 /CE = /EEROM           (GAL 26C, 0xFD0000)
//
//  The part holds 2 KB of odd bytes at 0xFD0001, 0xFD0003 ... 0xFD0FFF, and
//  because A12 reaches neither the GAL nor the part it mirrors twice through
//  0xFD1FFF.  An access takes 2 wait states (the 28R counter loads 13), which
//  is badlands_dtack's business, not this module's.
//
//  ---- the unlock / lock protocol -----------------------------------------
//    22J F74 half 1:  D(2)   = GND
//                     CLK(3) = /WRL      (rising = the end of any 68000 write)
//                     /PRE(4)= /UNLOCK   (a write to 0xFEE001, LS138 24C Y7)
//                     /CLR(1)= /RES
//                     Q(5)  -> 19J LS08 pin 5
//    19J LS08:        pin 5 = 22J Q , pin 4 = /RES , pin 6 = 27J /OE
//      =>  27J /OE = Q AND /RES
//
//   | state                            | Q | /OE | effect                     |
//   | after /RES                       | 0 |  0  | locked: outputs on, the    |
//   |                                  |   |     | part ignores /WE, so reads |
//   |                                  |   |     | work and writes cannot     |
//   | after a write to 0xFEE001 (/PRE) | 1 |  1  | unlocked: /OE forced high, |
//   |                                  |   |     | the next /WE programs a    |
//   |                                  |   |     | byte, reads are impossible |
//   | at the end of the next write     | 0 |  0  | re-locked, automatically   |
//
//  So: write anything to 0xFEE001, then write the data byte to 0xFD0001 + 2n,
//  and that write's own /WRL rising edge re-locks the part.  /PRE is level
//  sensitive and dominates the clock, so the unlock write does not clear
//  itself.
//
//  Because /WRL is not address-qualified, any 68000 write anywhere re-locks
//  the part.  An interrupt taken between the unlock and the data write -- its
//  stack frame is three /RAM writes -- therefore consumes the unlock and the
//  data byte is silently not stored.  That is real hardware behaviour, and it
//  is why the program masks interrupts around the pair.  A 68000 `RESET`
//  instruction does not re-lock the part: R58/R59 are series resistors and
//  cannot pull the /RES net down.
//
//  ---- programming time ---------------------------------------------------
//  The 2816A data sheet gives a self-timed byte write of typically 5 ms and at
//  most 10 ms, with the I/O pins high-Z throughout; MAME models no write time
//  at all.  The default here is the 10 ms bound, as the WRITE_CYCLES parameter
//  so that a simulation need not run half a million clocks per byte.
//
//  ---- the erased state, and why it is established here -------------------
//  The MRA carries no NVRAM element and the EEPROM is not part of the index-0
//  download image: the part ships erased and the game writes its own defaults
//  from the self test.  On the FPGA an inferred RAM powers up at 0x00, which
//  is not the erased state -- a factory 2816A reads 0xFF -- so the erase is
//  done here, at the end of every init window in which no image arrived (or in
//  which an all-zero image did: a `.nvm` the HPS has never written).
//  `programmed` is active high and resetless, and must never be cleared by
//  `init_reset`, which also pulses on every OSD reset -- clearing it there
//  would erase a restored image or a freshly set option.  One erase pass is
//  2048 clk_sys = 36 us against the ~145 ms of POR plus 8 frames that the
//  board holds the 68000 for, so it always finishes first.
//
//  ---- how the 2 KB array is stored on the FPGA ---------------------------
//  One write port and two registered read ports, which is the only shape
//  Quartus turns into block RAM on this family:
//
//    write : the loader byte, the power-up erase and the accepted 68000 write,
//            multiplexed in that priority
//    read 1: `cpu_addr`  -> `cpu_rdata`, the 68000's D7:0
//    read 2: `dump_addr` -> `dump_data`, the NVRAM upload channel
//
//  Quartus duplicates the array once per read port, so this is 2 x 2 M10K
//  blocks for 16 kbit -- the same thing it does for `badlands_cram` and
//  `badlands_lb`.  Both reads are one clk_sys behind their address, so
//  `dump_data` is valid the clock after `dump_addr`; `badlands_nvram_io`
//  drives `dump_addr` straight from `ioctl_addr`, which `sys/hps_io.sv` holds
//  for a whole HPS `io_strobe` beat before it samples `ioctl_din`, so the
//  upload stream is byte for byte what a combinatorial read would give.
//
//  The three writers cannot collide: `load_we` only pulses while an
//  `ioctl_index == 2` download is running, the power-up erase runs for 2048
//  clocks out of `init_reset` with the 68000 still in POR, and a
//  `write_accepted` needs the 68000 out of reset.
//============================================================================

module badlands_eeprom_2816 #(
	parameter int CLK_HZ       = 57272724,
	parameter int WRITE_CYCLES = CLK_HZ / 100      // 10 ms, the data-sheet bound
)(
	input  logic        clk,
	input  logic        init_reset,   // FPGA / PLL init and NVRAM restore
	input  logic        reset,        // the board /RES net, active high

	// /UNLOCK is the 22J flop's /PRE and is level sensitive, so this port takes
	// the level "a write to 0xFEE001 is in progress", not a pulse.  The same
	// cycle also carries `any_write` (the /WRL edge of that very write), and
	// the priority below lets /PRE win, exactly as the part does.
	input  logic        unlock,       // level: /UNLOCK asserted (0xFEE001 write)
	input  logic        any_write,    // 1-clk pulse: /WRL rising -- any 68000 write
	input  logic        cpu_we,       // 1-clk pulse: a write with /EEROM asserted
	input  logic [10:0] cpu_addr,     // 68000 A11:A1
	input  logic  [7:0] cpu_wdata,    // D7:0
	output logic  [7:0] cpu_rdata,    // D7:0
	output logic        busy,
	output logic        oe_n,         // 19J LS08 pin 6 -> 27J pin 20
	output logic        unlocked,     // 22J Q
	output logic        write_accepted,

	// MiSTer NVRAM restore / save-back (badlands_nvram_io)
	input  logic        load_we,
	input  logic [10:0] load_addr,
	input  logic  [7:0] load_data,
	input  logic [10:0] dump_addr,
	output logic  [7:0] dump_data
);

	localparam int BUSY_W = (WRITE_CYCLES <= 1) ? 1 : $clog2(WRITE_CYCLES + 1);
	logic [BUSY_W-1:0] busy_ctr;
	logic [7:0] mem [0:2047];

	// ---- power-up erase --------------------------------------------------
	// The explicit power-up values on these four and the `preserve` on
	// `programmed` are both load bearing.  They are resetless on purpose (the
	// header says why `programmed` must survive `init_reset`), but
	// `programmed`'s data input is the constant 1, so without `preserve`
	// Analysis & Synthesis deletes it as stuck at VCC; `~programmed` is then a
	// constant 0, `por_run` never sets and the erase never runs on the device.
	// Simulation cannot see that on its own -- a resetless register is X there
	// and `if (por_we)` takes the else branch -- so the initial values are what
	// make simulation and the device agree.
	// (PROCASSINIT/MULTIDRIVEN are Verilator objecting to exactly this pattern,
	// a declaration initial value on a register a process also writes, which is
	// the point here; they are waived on these four only.)
	/* verilator lint_off MULTIDRIVEN */
	/* verilator lint_off PROCASSINIT */
	(* preserve *) logic programmed = 1'b0;
	logic        ld_seen    = 1'b0;
	logic        ld_nonzero = 1'b0;
	logic        por_run    = 1'b0;
	/* verilator lint_on PROCASSINIT */
	/* verilator lint_on MULTIDRIVEN */
	logic [10:0] por_addr;

`ifndef ALTERA_RESERVED_QIS
	// Simulation only, and it models the FPGA rather than the part: an inferred
	// RAM powers up at 0x00, which is not a 2816A's erased state.  The erase
	// pass below turns it into the factory 0xFF, in simulation exactly as on
	// the device.
	initial for (int i = 0; i < 2048; i++) mem[i] = 8'h00;
`endif

	wire         ld_data_nz = load_we & (|load_data);
	wire         nx_seen    = ld_seen    | load_we;
	wire         nx_nonzero = ld_nonzero | ld_data_nz;
	wire         por_we     = por_run & ~load_we;

	always_ff @(posedge clk) begin
		if (ld_data_nz || write_accepted) programmed <= 1'b1;

		if (init_reset) begin
			ld_seen    <= nx_seen;
			ld_nonzero <= nx_nonzero;
			por_run    <= nx_seen ? ~nx_nonzero : ~programmed;
			por_addr   <= 11'd0;
		end else begin
			ld_seen    <= 1'b0;
			ld_nonzero <= 1'b0;
			if (load_we || write_accepted) por_run <= 1'b0;
			else if (por_run) begin
				por_addr <= por_addr + 1'b1;
				if (&por_addr) por_run <= 1'b0;
			end
		end
	end

	assign busy      = |busy_ctr;

	// 19J LS08: /OE = Q AND /RES.  Locked (Q = 0) or in reset -> /OE low, so
	// the outputs are on and the part ignores /WE.
	assign oe_n = unlocked & ~reset;

	// A write only takes effect while the part is unlocked (/OE high) and not
	// already programming.  `write_accepted` samples `unlocked` before the same
	// /WRL edge re-locks it, so the byte the game unlocked for does get stored.
	assign write_accepted = cpu_we & unlocked & ~busy;

	// ---- 22J F74 half 1 --------------------------------------------------
	// /CLR = /RES is asynchronous on the part; /UNLOCK (/PRE) is level
	// sensitive and beats the /WRL edge of its own cycle.
	always_ff @(posedge clk) begin
		if (init_reset || reset) unlocked <= 1'b0;
		else if (unlock)         unlocked <= 1'b1;
		else if (any_write)      unlocked <= 1'b0;
	end

	// ---- the write decode ------------------------------------------------
	// One enable per writer, in priority order:
	//     load_we  >  init_reset (no write at all)  >  por_we  >  write_accepted
	wire we_load = load_we;
	wire we_por  = ~load_we & ~init_reset & por_we;
	wire we_cpu  = ~load_we & ~init_reset & ~por_we & write_accepted;

	// ---- the programming timer -------------------------------------------
	always_ff @(posedge clk) begin
		if (load_we || init_reset) busy_ctr <= '0;
		else begin
			if (busy)      busy_ctr <= busy_ctr - 1'b1;
			if (we_cpu)    busy_ctr <= BUSY_W'(WRITE_CYCLES);
		end
	end

	// ---- the cell array: one write port, two registered read ports --------
	// It has no reset pin, so programming continues across a board reset.  The
	// loader write is deliberately outside the init_reset guard: every restored
	// byte arrives while init_reset is high, and guarding it there throws the
	// whole image away.
	//
	// The three writers are multiplexed into one port, in the priority of the
	// `if / else if` chain above, and both reads are registered.  That shape is
	// not a preference, it is the only one Quartus infers: a Cyclone V M10K
	// cannot be inferred from two independent write ports plus two reads, nor
	// from an asynchronous read port, and either alternative builds the whole
	// 2 KB out of flip-flops instead.
	wire        mem_we = we_load | we_por | we_cpu;
	wire [10:0] mem_wa = we_load ? load_addr : we_por ? por_addr : cpu_addr;
	wire  [7:0] mem_wd = we_load ? load_data : we_por ? 8'hFF    : cpu_wdata;

	logic [7:0] cpu_q;

	always_ff @(posedge clk) begin
		if (mem_we) mem[mem_wa] <= mem_wd;
		cpu_q     <= mem[cpu_addr];    // read port 1 -- the 68000 (D7:0)
		dump_data <= mem[dump_addr];   // read port 2 -- the NVRAM upload
	end

	// The I/O pins float while a byte is programming, and again while /OE is
	// high (the unlocked window, where reads are not possible).  There are no
	// pull-ups on D7:0, so the float is modelled as the 0xFF this core uses for
	// an undriven bus everywhere else.  The override is registered alongside
	// the RAM output, so `cpu_rdata` stays one clk_sys behind `cpu_addr`.
	logic float_d;
	always_ff @(posedge clk) float_d <= busy | oe_n;
	assign cpu_rdata = float_d ? 8'hFF : cpu_q;

endmodule
