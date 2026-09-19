`timescale 1ns/1ps
//============================================================================
//  Bad Lands MAIN BOARD -- the 68000 side of schematic sheets 1 and 2, plus
//  the memory path, tied together.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Structure ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_main.sv` (GPL-3.0, same
//  author); every strobe, lane and wait below is Bad Lands' own.
//
//  Here: the 68000, the decode, the wait sequencer, the 22V video/work RAM,
//  the interrupt latch, the watchdog and POR, the inputs, the EEPROM, the
//  program ROM and the whole graphics-SDRAM path.  Not here: the SOS-2 raster
//  (`h`, `v` and `vblank` come in), the colour RAM, the playfield and motion
//  object engines, and the sound section.
//
//  Reset tree -- `/RES` (the 28A LS197 QD) reaches exactly four loads:
//     68000 /HALT and /RESET (through the R58/R59 1 K series resistors)
//     22J F74 half 1 /CLR    -- the EEPROM unlock flop
//     19J LS08 pins 4 and 10 -- the EEPROM /OE gate and the /AUDRES gate
//  It does not reach the SOS-2, the LETA, the 18S playfield bank flop, the MO
//  and priority flops, the PIXCLK flop, the sound IRQ latch, either mailbox
//  flag or the sound section, so `res_n` goes out for those blocks to see and
//  is wired only to the four loads here.  A 68000 `RESET` instruction is
//  observable nowhere -- R58/R59 cannot pull a totem-pole LS197 output down --
//  so `badlands_cpu.reset_inst` is left unconnected on purpose.
//
//  The undriven data bus.  Several reads drive only part of it:
//     /MISC2 read  -- GAL 9N drives D8 only
//     /SWITCHES    -- 18C LS244 drives D7:0 only
//     /LETA        -- the LETA drives D7:0 only
//     /AUDRD       -- 13F LS374 drives D15:8 only
//     /CRAM        -- 8F LS245 drives D15:8 only
//     /EEROM       -- the 2816A drives D7:0 only
//     anything undecoded, including 0x070000
//  The board has no data-bus pull-ups (its one 1 K pull-up is shared by /BERR,
//  /BR and /BGACK), so a real floating bus returns the last driven value.
//  `UNDRIVEN_DATA = 0xFFFF` is the default because that is what MAME returns;
//  `UNDRIVEN_LAST_VALUE = 1` models the board instead.  The program masks
//  every one of these fields away.
//
//  0x070000 needs no special case.  Every fault vector points at 0x000300,
//  whose only instruction is `tst.l $00070000`, and the VBLANK handler holds a
//  `jmp $00070000`.  A23 = 0 there, so /RAM, /EEROM and /AUDRESD are all
//  inactive and the 28R LS163A loads V = 15, where RCO asserts /DTACK at once:
//  zero wait states, the cycle completes, the 68000 does not hang and the
//  watchdog is not invoked.  The read returns UNDRIVEN_DATA and the program
//  falls through 258 zero bytes from 0x306 to 0x406, resuming at 0x40A.
//============================================================================

module badlands_main #(
	parameter int  CLK_HZ              = 57272724,
	// what a read of an undriven lane returns; see the header
	parameter logic [15:0] UNDRIVEN_DATA = 16'hFFFF,
	parameter bit  UNDRIVEN_LAST_VALUE = 1'b0,
	parameter int  POR_US              = 11000,
	// LETA decode factor, steering sense and pedal rates
	parameter bit  LETA_DECODE_X2      = 1'b0,
	parameter bit  QUAD_INVERT         = 1'b0,
	parameter int  PEDAL_IDLE_HZ       = 283,
	parameter int  PEDAL_PRESSED_HZ    = 7550,
	parameter bit  PEDAL_IDLE_SILENT   = 1'b0,
	// the 2816A's self-timed byte write
	parameter int  EEPROM_WRITE_CYCLES = CLK_HZ / 100
)(
	input  logic        clk,            // clk_sys 57.272724 MHz
	input  logic        ce_7m,          // clk_sys/8, the `7M` rising edge
	input  logic        ce_14m,         // clk_sys/4
	input  logic        init_reset,     // ~pll_locked only
	input  logic        ext_reset,      // OSD reset; re-runs the POR

	// ---------------- the raster, from badlands_sos2_sync ---------------
	input  logic  [8:0] h,
	input  logic  [8:0] v,
	input  logic        vblank,         // SOS-2 pin 22, raw active high

	// ---------------- VRAM video port -----------------------------------
	output logic [15:0] ramd,
	output logic        ramd_pf_stb,
	output logic        ramd_mo0_stb,
	output logic        ramd_mo1_stb,

	// ---------------- CRAM CPU port -------------------------------------
	output logic        cram_cs,        // /CRAM asserted (level, follows /AS)
	output logic  [9:1] cram_a,         // 68000 A9:1
	output logic        cram_we,        // 1 clk_sys pulse: write the byte
	output logic  [7:0] cram_din,       // the 68000's D15:8
	input  logic  [7:0] cram_dout,      // combinational from cram_a

	// ---------------- the 18S playfield bank flop (24C Y6) --------------
	output logic        pfbank,         // D0 of the last write to 0xFEC001
	output logic        pfbank_we,      // 1 clk_sys pulse

	// ---------------- sound mailbox -------------------------------------
	output logic        audwr_stb,      // 1 clk pulse at the end of a 0xFE8000 write
	output logic  [7:0] audwr_data,     // the 68000's D15:8 for that write
	output logic        audrd_stb,      // 1 clk pulse at the end of a 0xFEA000 read
	input  logic  [7:0] audrd_data,     // 13F LS374 -> D15:8
	// GAL 9N pin 12 sits in the video block on the real board (sheet 4), so the
	// D8 = AUDFULL read at 0xFC0000 comes in here rather than being formed
	// locally; `misc2_n` and `cpu_rw` go out to that same GAL.
	input  logic        audfull_d8,     // 9N pin 12 value      (= AUDFULL)
	input  logic        audfull_d8_oe,  // 9N pin 12 enable     (= /MISC2 & R/W)
	output logic        misc2_n,        // -> badlands_video's 9N pin 13
	input  logic        audirq_n,       // 15J sound->main flag -> /IPL1
	output logic        audres,         // active high: a write to 0xFC0000

	output logic        snd_rom_wr,     // the sound board's 64 KB BRAM
	output logic [15:0] snd_rom_addr,
	output logic  [7:0] snd_rom_data,

	// ---------------- gfx byte clients ----------------------------------
	input  logic        pf_req,
	input  logic [18:0] pf_addr,
	output logic        pf_ack,
	output logic  [7:0] pf_data,
	input  logic        mo_req,
	input  logic [17:0] mo_addr,
	output logic        mo_ack,
	output logic  [7:0] mo_data,

	// ---------------- SDRAM pins ----------------------------------------
	inout  wire  [15:0] SDRAM_DQ,
	output logic [12:0] SDRAM_A,
	output logic  [1:0] SDRAM_BA,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_CKE,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE,

	// ---------------- MiSTer HPS ioctl ----------------------------------
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,
	input  logic        ioctl_upload,
	output logic        ioctl_wait,
	output logic        ioctl_upload_req,
	output logic  [7:0] ioctl_upload_index,
	output logic  [7:0] ioctl_din,

	// ---------------- controls ------------------------------------------
	input  logic        test_n,         // /TEST, active low
	input  logic        p1_fire,        // left  start/fire, active high here
	input  logic        p2_fire,        // right start/fire
	input  logic  [3:0] sw1_n,          // SW1 4:1 -> D3:D0, active low
	input  logic signed [8:0] wheel0_delta,
	input  logic        wheel0_valid,
	input  logic signed [8:0] wheel1_delta,
	input  logic        wheel1_valid,
	input  logic        pedal0,
	input  logic        pedal1,
	input  logic        pedal_invert,

	// ---------------- observability -------------------------------------
	output logic        res_n,          // the /RES net
	output logic        rom_loaded,
	output logic [23:1] cpu_a,
	output logic  [2:0] cpu_fc,      // 111 = CPU space = an interrupt acknowledge
	output logic        cpu_as,
	output logic        cpu_rw,
	output logic        cpu_uds_n,
	output logic        cpu_lds_n,
	output logic [15:0] cpu_wdata,
	output logic [15:0] cpu_rdata,
	output logic        cpu_dtack
);

	// =====================================================================
	//  the 68000 and its decode
	// =====================================================================
	logic [15:0] rdata;
	logic        as, uds_n, lds_n, rw, dtack, vpa_n, ce_7m_n;
	logic  [2:0] fc, ipl;
	logic [23:1] a;
	logic [15:0] wdata;

	/* verilator lint_off UNUSEDSIGNAL */
	logic        reset_inst;   // observable nowhere on this board; see the header
	/* verilator lint_on UNUSEDSIGNAL */

	// The CPU is held while the board's /RES is asserted, while the PLL is not
	// locked, and until the ROMs have arrived.
	wire cpu_reset = init_reset | ~rom_loaded | ~res_n;

	badlands_cpu u_cpu (
		.clk(clk), .ce_7m(ce_7m), .reset(cpu_reset), .ipl(ipl),
		.a(a), .wdata(wdata), .rdata(rdata),
		.as(as), .uds_n(uds_n), .lds_n(lds_n), .rw(rw),
		.dtack(dtack), .vpa_n(vpa_n), .fc(fc),
		.reset_inst(reset_inst), .ce_7m_n(ce_7m_n)
	);

	// /MISC only enables the LS138 24C, whose eight Y outputs are the strobes
	// this module actually uses; a13s/a14s are the empty SLAPSTIC socket's
	// pass-throughs and are already folded into `rom_a`, and rom_oe_lo/hi pick
	// which 27512 pair answers, which is the A17 the flat BRAM image carries.
	/* verilator lint_off UNUSEDSIGNAL */
	logic misc_n, a13s, a14s;
	logic rom_oe_lo_n, rom_oe_hi_n;
	/* verilator lint_on UNUSEDSIGNAL */
	logic eerom_n, cram_n, ram_n;
	logic wdog_n, virqack_n, switches_n, leta_n, audwr_n, audrd_n,
	      pfbank_n, unlock_n;
	logic rom_ce_n;
	logic [15:0] rom_a;

	badlands_addr_decode u_dec (
		.as(as), .rw(rw), .a(a),
		.misc2_n(misc2_n), .misc_n(misc_n), .eerom_n(eerom_n),
		.cram_n(cram_n), .ram_n(ram_n), .a13s(a13s), .a14s(a14s),
		.wdog_n(wdog_n), .virqack_n(virqack_n), .switches_n(switches_n),
		.leta_n(leta_n), .audwr_n(audwr_n), .audrd_n(audrd_n),
		.pfbank_n(pfbank_n), .unlock_n(unlock_n),
		.rom_ce_n(rom_ce_n), .rom_oe_lo_n(rom_oe_lo_n),
		.rom_oe_hi_n(rom_oe_hi_n), .rom_a(rom_a)
	);

	// 24J F20: /VPA = NOT(AS & FC2 & FC1 & FC0).  FC = 7 is CPU space, which the
	// 68000 drives only for an interrupt acknowledge, so every interrupt on this
	// board is autovectored.
	assign vpa_n = ~(as & fc[2] & fc[1] & fc[0]);

	// 17J F32: /AUDRESD = R/W + /MISC2 -- low only on a write to 0xFC0000.  It
	// is both the sound reset and the DTACK counter's 5-wait load.
	wire audresd_n = rw | misc2_n;
	assign audres  = ~audresd_n;

	// =====================================================================
	//  the end-of-cycle strobes
	// =====================================================================
	// 17J F32 also makes /WRL = R/W + /AS, low for the whole of any 68000 write
	// and rising when /AS negates.  Every "the write has happened" event on this
	// board is that rising edge, so the decode is sampled while /AS is still
	// asserted and used at the edge.
	logic as_q, rw_q, uds_q;
	logic q_audwr, q_audrd, q_eerom, q_cram, q_pfbank, q_unlock;
	always_ff @(posedge clk) begin
		as_q <= as;
		if (as) begin
			rw_q     <= rw;
			q_audwr  <= ~audwr_n;
			q_audrd  <= ~audrd_n;
			q_eerom  <= ~eerom_n;
			q_cram   <= ~cram_n;
			q_pfbank <= ~pfbank_n;
			q_unlock <= ~unlock_n;
		end
		// The 68000 asserts /UDS //LDS one clock after /AS on a write, so the
		// lane is collected stickily over the cycle -- cleared as /AS asserts,
		// set the moment /UDS does.  Only /UDS is kept: it is the only lane any
		// strobe on this board looks at (the 13J F00; see `cram_we` below).  A
		// level copy would be wrong: /UDS and /AS both negate at S7 and nothing
		// guarantees which clk_sys cycle sees it first.
		if (as && !as_q)      uds_q <= 1'b0;
		else if (as && !uds_n) uds_q <= 1'b1;
	end
	wire as_fall = ~as & as_q;
	wire wrl_rise = as_fall & ~rw_q;       // /WRL rising = the end of a write
	wire rd_end   = as_fall &  rw_q;

	// /UNLOCK must still be asserted in the cycle `wrl_rise` fires.  On the
	// board /UNLOCK is the 22J flop's /PRE and /WRL its clock, and both release
	// when /AS negates; /PRE is level sensitive and dominates the clock, so the
	// unlock write does not clear itself.  `unlock_n` is combinational from
	// `as` and is therefore already high when the edge detector sees the fall,
	// so the level is extended over that one cycle.  Without it the EEPROM
	// accepts no writes at all.
	wire unlock_lvl = ~unlock_n | (as_fall & q_unlock);

	// =====================================================================
	//  the 22V video / work RAM
	// =====================================================================
	logic [15:0] vram_dout;
	logic        rtime_n;
	/* verilator lint_off UNUSEDSIGNAL */
	logic        utime_n, ramre_n, ramwe_n, ramoe_n, ramwp_n;   // 21R pins, unused here
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_vram u_vram (
		.clk(clk), .reset(init_reset), .ce_7m(ce_7m), .ce_14m(ce_14m),
		.h(h), .v(v),
		.ramd(ramd), .ramd_pf_stb(ramd_pf_stb),
		.ramd_mo0_stb(ramd_mo0_stb), .ramd_mo1_stb(ramd_mo1_stb),
		// /RAM arrives with no /AS term, exactly as GAL 26C emits it
		.as(as), .rw(rw), .uds_n(uds_n), .lds_n(lds_n), .ram_n(ram_n),
		.cpu_a(a[12:1]), .cpu_din(wdata), .cpu_dout(vram_dout),
		.rtime_n(rtime_n),
		.utime_n(utime_n), .ramre_n(ramre_n), .ramwe_n(ramwe_n),
		.ramoe_n(ramoe_n), .ramwp_n(ramwp_n)
	);

	// =====================================================================
	//  the 28R LS163A wait sequencer
	// =====================================================================
	badlands_dtack u_dtack (
		.clk(clk), .ce_7m_n(ce_7m_n),
		.as(as), .ram_n(ram_n), .audresd_n(audresd_n), .eerom_n(eerom_n),
		.vpa_n(vpa_n), .rtime_n(rtime_n), .dtack(dtack)
	);

	// =====================================================================
	//  interrupts, watchdog and POR
	// =====================================================================
	/* verilator lint_off UNUSEDSIGNAL */
	logic virq, por;
	logic [3:0] wdog_count;
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_irq u_irq (
		.clk(clk),
		// /RES does not reach 22J half 2, so this is power-up only
		.reset(init_reset),
		.vblank(vblank), .virqack_n(virqack_n), .audirq_n(audirq_n),
		.ipl(ipl), .virq(virq)
	);

	badlands_watchdog #(.CLK_HZ(CLK_HZ), .POR_US(POR_US)) u_wdog (
		.clk(clk),
		.ext_reset(init_reset | ext_reset | ~rom_loaded),
		.vblank(vblank), .wdog_n(wdog_n),
		.wdog_jumper(1'b0),               // the shipped build
		.res_n(res_n), .por(por), .count(wdog_count)
	);

	// =====================================================================
	//  inputs
	// =====================================================================
	logic [7:0] switches_dout, leta_dout;
	/* verilator lint_off UNUSEDSIGNAL */
	logic [7:0] leta_c0, leta_c1, leta_c2, leta_c3;
	/* verilator lint_on UNUSEDSIGNAL */

	badlands_inputs #(
		.CLK_HZ(CLK_HZ), .LETA_DECODE_X2(LETA_DECODE_X2),
		.QUAD_INVERT(QUAD_INVERT), .PEDAL_IDLE_HZ(PEDAL_IDLE_HZ),
		.PEDAL_PRESSED_HZ(PEDAL_PRESSED_HZ),
		.PEDAL_IDLE_SILENT(PEDAL_IDLE_SILENT)
	) u_inputs (
		.clk(clk), .reset(init_reset),
		.h1(h[0]),                        // the LETA's CK is /1H
		.test_n(test_n), .vblank(vblank),
		.p1_fire(p1_fire), .p2_fire(p2_fire), .sw1_n(sw1_n),
		.switches_dout(switches_dout),
		.leta_a(a[2:1]), .leta_dout(leta_dout),
		.wheel0_delta(wheel0_delta), .wheel0_valid(wheel0_valid),
		.wheel1_delta(wheel1_delta), .wheel1_valid(wheel1_valid),
		.pedal0(pedal0), .pedal1(pedal1), .pedal_invert(pedal_invert),
		.leta_count0(leta_c0), .leta_count1(leta_c1),
		.leta_count2(leta_c2), .leta_count3(leta_c3)
	);

	// =====================================================================
	//  EEPROM + NVRAM
	// =====================================================================
	logic [7:0] ee_rdata;
	/* verilator lint_off UNUSEDSIGNAL */
	logic       ee_busy, ee_oe_n, ee_unlocked;
	/* verilator lint_on UNUSEDSIGNAL */
	logic       ee_write_accepted;
	logic       nv_load_we;
	logic [10:0] nv_load_addr, nv_dump_addr;
	logic  [7:0] nv_load_data, nv_dump_data;

	// The EEPROM's init window: the PLL lock, the ROM download, and an index-2
	// NVRAM restore.  Every restored byte must arrive inside it.
	wire ee_init = init_reset | ~rom_loaded |
	               (ioctl_download & (ioctl_index == 16'd2));

	badlands_eeprom_2816 #(
		.CLK_HZ(CLK_HZ), .WRITE_CYCLES(EEPROM_WRITE_CYCLES)
	) u_eeprom (
		.clk(clk), .init_reset(ee_init),
		.reset(~res_n),                   // /RES reaches the 22J /CLR
		.unlock(unlock_lvl),              // /UNLOCK is a level on /PRE
		.any_write(wrl_rise),             // /WRL rising, not address qualified
		.cpu_we(wrl_rise & q_eerom),
		.cpu_addr(a[11:1]), .cpu_wdata(wdata[7:0]), .cpu_rdata(ee_rdata),
		.busy(ee_busy), .oe_n(ee_oe_n), .unlocked(ee_unlocked),
		.write_accepted(ee_write_accepted),
		.load_we(nv_load_we), .load_addr(nv_load_addr), .load_data(nv_load_data),
		.dump_addr(nv_dump_addr), .dump_data(nv_dump_data)
	);

	badlands_nvram_io u_nvram (
		.clk(clk), .init_reset(init_reset),
		.write_accepted(ee_write_accepted),
		.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
		.ioctl_index(ioctl_index), .ioctl_upload(ioctl_upload),
		.load_we(nv_load_we), .load_addr(nv_load_addr), .load_data(nv_load_data),
		.dump_addr(nv_dump_addr), .dump_data(nv_dump_data),
		.ioctl_upload_req(ioctl_upload_req),
		.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din)
	);

	// =====================================================================
	//  program ROM and the download path
	// =====================================================================
	logic        prog_rom_wr;
	logic [17:0] prog_rom_addr;
	logic  [7:0] prog_rom_data;
	logic [15:0] prog_rom_q;
	logic        tiles_wr, sprites_wr;
	logic [18:0] tiles_addr;
	logic [17:0] sprites_addr;
	logic  [7:0] gfx_data;

	badlands_rom_loader u_ldr (
		.clk(clk), .reset(init_reset),
		.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
		.ioctl_index(ioctl_index),
		.prog_rom_wr(prog_rom_wr), .prog_rom_addr(prog_rom_addr),
		.prog_rom_data(prog_rom_data),
		.snd_rom_wr(snd_rom_wr), .snd_rom_addr(snd_rom_addr),
		.snd_rom_data(snd_rom_data),
		.tiles_wr(tiles_wr), .tiles_addr(tiles_addr),
		.sprites_wr(sprites_wr), .sprites_addr(sprites_addr),
		.gfx_data(gfx_data), .rom_loaded(rom_loaded)
	);

	// The ROM address is {A16,A15,A14S,A13S,A12:A1} and A17 picks which 27512
	// pair answers, so the flat 128 K-word BRAM index is {A17, rom_a}.
	badlands_prog_rom u_prom (
		.clk(clk), .wr(prog_rom_wr), .wr_addr(prog_rom_addr),
		.wr_data(prog_rom_data),
		.rd_a({a[17], rom_a}), .rd_data(prog_rom_q)
	);

	logic        dl_wr, dl_ack, sdl_busy;
	logic [23:0] dl_waddr;
	logic [15:0] dl_wdata;

	badlands_sdram_loader u_sdl (
		.clk(clk), .reset(init_reset),
		.tiles_wr(tiles_wr), .tiles_addr(tiles_addr),
		.sprites_wr(sprites_wr), .sprites_addr(sprites_addr),
		.ld_data(gfx_data), .wr_busy(sdl_busy),
		.dl_wr(dl_wr), .dl_waddr(dl_waddr), .dl_wdata(dl_wdata), .dl_ack(dl_ack)
	);

	assign ioctl_wait = sdl_busy;

	logic        sd_req, sd_we, sd_ready, sd_valid, rfsh_ok;
	logic [23:0] sd_addr;
	logic  [1:0] sd_blen;
	logic [15:0] sd_wdata, sd_rdata;

	badlands_gfx_mem u_gfx (
		.clk(clk), .reset(init_reset),
		.pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack), .pf_data(pf_data),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),
		.h_mod8(h[2:0]), .rfsh_ok(rfsh_ok),
		.dl_wr(dl_wr), .dl_waddr(dl_waddr), .dl_wdata(dl_wdata), .dl_ack(dl_ack),
		.sd_req(sd_req), .sd_addr(sd_addr), .sd_we(sd_we), .sd_blen(sd_blen),
		.sd_wdata(sd_wdata), .sd_ready(sd_ready), .sd_valid(sd_valid),
		.sd_rdata(sd_rdata)
	);

	badlands_sdram u_sdram (
		.clk(clk),
		// This must be ~pll_locked only: a reset here during the download drops
		// CKE, restarts the 200 us init mid-stream and silently loses every
		// loader write already made.
		.reset(init_reset),
		.addr(sd_addr), .wdata(sd_wdata), .we(sd_we), .blen(sd_blen),
		.req(sd_req), .rfsh_ok(rfsh_ok),
		.rdata(sd_rdata), .valid(sd_valid), .ready(sd_ready),
		.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
		.SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_CKE(SDRAM_CKE),
		.SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE)
	);

	// =====================================================================
	//  the CRAM CPU port
	// =====================================================================
	assign cram_cs  = ~cram_n;
	assign cram_a   = a[9:1];
	assign cram_din = wdata[15:8];        // 8F LS245 drives D15:8 only
	// The write strobe carries /UDS.  The 8K SRAM's /WE is
	//     /CRAMOE = CRAM AND W/R                 (19J LS08, 13,12 -> 11)
	//     /CRAMWP = NAND( UDS , /CRAMOE )        (13J F00,  13,12 -> 11)
	// The part is one byte wide on D15:8 through 8F, so a lower-byte write --
	// an odd address, /LDS alone -- must not reach it at all.  Nothing in GAL
	// 26C or the LS138 looks at the lanes, which is why the qualification has
	// to be here: the program loads the palette as pairs of byte writes, an
	// even byte carrying the colour and an odd byte carrying a constant the
	// board throws away, and without this term the constant overwrites every
	// colour byte one bus cycle later.
	assign cram_we  = wrl_rise & q_cram & uds_q;

	// =====================================================================
	//  the 18S playfield bank flop  (24C Y6 /PFBANK, D0)
	// =====================================================================
	// /RES does not reach 18S, so it is cleared only at FPGA init.
	assign pfbank_we = wrl_rise & q_pfbank;
	always_ff @(posedge clk) begin
		if (init_reset)   pfbank <= 1'b0;
		else if (pfbank_we) pfbank <= wdata[0];
	end

	// =====================================================================
	//  the sound mailbox
	// =====================================================================
	assign audwr_stb = wrl_rise & q_audwr;
	assign audrd_stb = rd_end   & q_audrd;
	// The 68000 removes its write data at the end of the cycle, and audwr_stb
	// fires ON that edge, so D15:8 is held from while /AS was still asserted.
	logic [7:0] wdata_hi_q;
	always_ff @(posedge clk) if (as) wdata_hi_q <= wdata[15:8];
	assign audwr_data = wdata_hi_q;       // 15F LS374 takes D15:8

	// =====================================================================
	//  the CPU read mux
	// =====================================================================
	logic [15:0] undriven;
	logic [15:0] last_bus;

	always_comb begin
		undriven = UNDRIVEN_LAST_VALUE ? last_bus : UNDRIVEN_DATA;

		rdata = undriven;
		if      (!rom_ce_n)        rdata = prog_rom_q;
		else if (!ram_n && as)     rdata = vram_dout;
		else if (!cram_n)          rdata = {cram_dout,     undriven[7:0]};
		else if (!eerom_n)         rdata = {undriven[15:8], ee_rdata};
		else if (!switches_n)      rdata = {undriven[15:8], switches_dout};
		else if (!leta_n)          rdata = {undriven[15:8], leta_dout};
		else if (!audrd_n)         rdata = {audrd_data,    undriven[7:0]};
		// D8 comes from GAL 9N pin 12, which is in the video block; its own
		// enable term is /MISC2 & R/W, i.e. exactly this branch's condition.
		else if (!misc2_n && rw)   rdata = {undriven[15:9],
		                                    audfull_d8_oe ? audfull_d8 : undriven[8],
		                                    undriven[7:0]};
	end

	// The physical alternative behind UNDRIVEN_LAST_VALUE: with no bus
	// pull-ups, a floating bus holds whatever was driven onto it last.
	always_ff @(posedge clk) begin
		if (init_reset)  last_bus <= UNDRIVEN_DATA;
		else if (as) begin
			if (!rw)                                       last_bus <= wdata;
			else if (!rom_ce_n || (!ram_n && as))          last_bus <= rdata;
		end
	end

	assign cpu_a     = a;
	assign cpu_fc    = fc;
	assign cpu_as    = as;
	assign cpu_rw    = rw;
	assign cpu_uds_n = uds_n;
	assign cpu_lds_n = lds_n;
	assign cpu_wdata = wdata;
	assign cpu_rdata = rdata;
	assign cpu_dtack = dtack;

endmodule
