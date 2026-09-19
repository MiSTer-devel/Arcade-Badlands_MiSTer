`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) — game logic top, below the MiSTer `emu` glue.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE and NOTICE.md.  Structure inherited from the Vindicators and
//  Xybots MiSTer cores (GPL-3.0, same author).
//
//  Three blocks, one per part of the PCB:
//
//    badlands_main   sheets 1 and 2 plus the whole memory path: fx68k 22E, the
//                    GAL 26C / LS138 24C decode, the 28R LS163A wait sequencer,
//                    the 8 K x 8 22V video/work RAM and its GAL 21R slot
//                    schedule, the 22J VBLANK latch and 13J IPL encoder, the
//                    POR + 28A LS197 watchdog, the LETA 20C with the steering
//                    and LM556 pedal models, the 2816A EEPROM + MiSTer NVRAM,
//                    the 256 KB program ROM, the ROM loader and the graphics
//                    SDRAM path
//    badlands_video  sheets 1 (SOS-2 25M, GAL 26R), 3 (playfield), 4 (motion
//                    objects, GAL 9N priority, the colour RAM) and 6 (the R2R
//                    DACs and composite sync)
//    badlands_sound  sheet 5 in full: the 6502A 14C on T65, the GAL 12E / LS138
//                    10F decode, the 6264 9B, the banked 27512 9C, the 10E /
//                    14E / 16C glue, the mailboxes, the YM2151 9D through jt51
//                    and the LM324/TDA2030 tail
//
//  `badlands_main` also CONTAINS `badlands_rom_loader`, `badlands_sdram_loader`,
//  `badlands_sdram`, `badlands_gfx_mem`, `badlands_prog_rom` and
//  `badlands_nvram_io`, so there is no separate memory wrapper: this file
//  instantiates three modules and the two steering front ends and does nothing
//  else but wire them.
//
//  ---- clocking -----------------------------------------------------------
//
//  ONE clock domain, clk_sys = 57.272724 MHz = 4 x the 14.318181 MHz crystal,
//  with integer clock enables and no fractional enables anywhere:
//
//     ce_14m = clk_sys / 4   = 14.318181 MHz  14M: CRAM byte rate, GAL 21R clock
//     ce_7m  = clk_sys / 8   =  7.159091 MHz  7M:  pixel, H counter, 68000
//     ce_1h  = clk_sys / 16  =  3.579545 MHz  1H:  YM2151 phiM, LETA CK
//     ce_2h  = clk_sys / 32  =  1.789772 MHz  2H:  6502 phi0
//
//  All four are decodes of ONE free-running 5-bit counter, so each slower enable
//  is a strict subset of every faster one.  On the board 1H and 2H are literally
//  the H counter's low bits, which is why the sound section's clocks are exact
//  sub-multiples of the pixel clock rather than a second oscillator.
//
//  PHASE CONVENTION (inherited from Xybots/Blasteroids/Vindicators): the clk_sys
//  cycle in which `ce_7m` is high is the LAST cycle of SOS-2 count `h`.
//
//  ---- the reset tree, and the ONE rule that must not be broken -----------
//    init_reset  ~pll_locked ONLY, STRAIGHT into `badlands_main.init_reset` and
//                nowhere else.  It is the only reset `badlands_sdram` may ever
//                see: a reset there during the download drops CKE, restarts the
//                200 us init mid-stream and silently loses every loader write
//                already made.
//    game_reset  reset | ~rom_loaded -- the OSD/system reset plus "the ROMs are
//                not in place yet".  It re-runs the main board's power-on
//                sequence through `ext_reset`.
//    vid_reset   init_reset | ~rom_loaded -- the FPGA power-on and the download
//                hold, and NOT the OSD reset, because of the /RES rule below.
//    /RES        formed INSIDE badlands_main by the 28A LS197 watchdog and the
//                power-on one-shot.  It reaches only the 68000, the EEPROM
//                unlock flop and the two 19J gates — NOT the SOS-2, NOT the
//                playfield bank flop, NOT the sound section.  The one place it
//                leaves the main board is the 19J AND that makes /AUDRES, which
//                is why `audres_n` below is `res_n & ~audres`.
//============================================================================

module badlands_core #(
	// Raster lines per frame.  One parameter, so the raster and the
	// analogue-alignment stage cannot drift apart.
	parameter int  V_TOTAL           = 262,
	// 0 keeps the LM556's idle rate; 1 silences the idle oscillator so the self
	// test reads 00.
	parameter bit  PEDAL_IDLE_SILENT = 1'b0,
	// The power-on one-shot, in microseconds.  Simulation may shorten it;
	// nothing else may.
	parameter int  POR_US            = 11000,
	// The 2816A's self-timed byte write, in clk_sys cycles: the data-sheet
	// 10 ms at 57.272724 MHz.  Simulation may shorten it, as with POR_US.
	parameter int  EEPROM_WRITE_CYCLES = 572727
)(
	input  logic        clk_sys,        // 57.272724 MHz
	input  logic        reset,          // active-high system/OSD reset
	input  logic        init_reset,     // ~pll_locked ONLY (SDRAM + loader)

	// ---- HPS ioctl: index 0 = the ROM stream, index 2 = the 2816A NVRAM ----
	input  logic        ioctl_download,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	input  logic [15:0] ioctl_index,
	output logic        ioctl_wait,
	input  logic        ioctl_upload,
	output logic        ioctl_upload_req,
	output logic  [7:0] ioctl_upload_index,
	output logic  [7:0] ioctl_din,

	// ---- controls: ACTIVE HIGH = "pressed" here; the board's active-low
	//      senses are formed inside badlands_inputs ------------------------
	input  logic        service,        // the 19B self-test slide switch, HIGH = ON
	input  logic  [3:0] sw1,            // SW1 1..4, HIGH = closed = ON
	input  logic        p1_fire,        // LEFT  Start/Fire  (JAM-22 + JAM-17)
	input  logic        p2_fire,        // RIGHT Start/Fire  (JAM-Z  + JAM-U)
	input  logic        coin1,          // JAM-16, HIGH = closed
	input  logic        coin2,          // JAM-T,  HIGH = closed
	input  logic        p1_pedal,       // the gas pedal, HIGH = pressed
	input  logic        p2_pedal,
	// Test switch: swaps the pedal CLK/DIR pair.  0 = the sense the board uses,
	// with the LETA pedal channels counting DOWN.
	input  logic        pedal_invert,

	// Every MiSTer control is summed into each wheel -- no source selector
	// (rtl/badlands_steer.sv).  Two feel settings, shared by both players: the
	// analog-stick rate, and the spinner/paddle/mouse sensitivity.
	input  logic  [1:0] steer_sens,     // stick: 0 Normal, 1 High, 2 Low, 3 Lowest
	input  logic  [3:0] enc_sens,       // encoders: OSD index, 0 = 1x
	input  logic signed [7:0] p1_stick_x,
	input  logic signed [7:0] p2_stick_x,
	input  logic  [7:0] p1_paddle,
	input  logic  [7:0] p2_paddle,
	input  logic  [8:0] p1_spinner,
	input  logic  [8:0] p2_spinner,
	input  logic        p1_mouse_stb,   // ps2_mouse[24]; P1 only, one mouse
	input  logic signed [8:0] p1_mouse_dx,
	input  logic        p1_left, p1_right,
	input  logic        p2_left, p2_right,

	// ---- video out (single clk_sys domain, qualified by ce_pix) ----
	output logic        ce_pix,
	output logic  [7:0] vga_r, vga_g, vga_b,
	output logic        hsync, vsync, hblank, vblank,

	// ---- audio (the board is MONO: both carry one signal) ----
	output logic signed [15:0] aud_l, aud_r,

	// ---- SDRAM chip ----
	inout  wire  [15:0] SDRAM_DQ,
	output logic [12:0] SDRAM_A,
	output logic  [1:0] SDRAM_BA,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_CKE,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE
);

	// =====================================================================
	//  clock enables
	// =====================================================================
	// Left unreset and uninitialised on purpose: Quartus powers it up at 0,
	// which fixes the enable PHASE without a reset net.
	logic [4:0] ce_cnt;
	always_ff @(posedge clk_sys) ce_cnt <= ce_cnt + 5'd1;

	wire ce_14m = (ce_cnt[1:0] == 2'd0);   // 14.318181 MHz
	wire ce_7m  = (ce_cnt[2:0] == 3'd0);   //  7.159091 MHz  pixel / 68000
	wire ce_1h  = (ce_cnt[3:0] == 4'd0);   //  3.579545 MHz  YM2151 / LETA
	wire ce_2h  = (ce_cnt      == 5'd0);   //  1.789772 MHz  6502

	// =====================================================================
	//  resets
	// =====================================================================
	logic rom_loaded;
	wire  game_reset = reset | ~rom_loaded;

	// The FPGA-level init and the download hold, and NOT the OSD reset: /RES
	// reaches nothing in the video block (see the header), so holding the reset
	// button on a real cabinet stops the CPU and leaves the raster running on
	// whatever the 22V still holds.  So does this.
	wire  vid_reset  = init_reset | ~rom_loaded;

	// =====================================================================
	//  the nets between the three boards
	// =====================================================================
	// raster taps (badlands_video -> badlands_main / badlands_sound)
	logic [8:0] h, v;
	logic       v32, vblank_raw, virq_edge, gfx_late;

	// the 22V VRAM video port
	logic [15:0] ramd;
	logic        ramd_pf_stb, ramd_mo0_stb, ramd_mo1_stb;

	// the two graphics byte clients: playfield and motion objects
	logic        pf_req, pf_ack, mo_req, mo_ack;
	logic [18:0] pf_addr;
	logic [17:0] mo_addr;
	logic  [7:0] pf_data, mo_data;

	// the colour RAM's CPU port
	logic        cram_cs, cram_we;
	logic  [9:1] cram_a;
	logic  [7:0] cram_din, cram_dout;

	// the 18S playfield bank flop
	logic        pfbank, pfbank_we;
	// `badlands_main` publishes the 18S flop's OUTPUT and a one-clk write pulse,
	// and the flop takes its new value ON that pulse's edge, so during the pulse
	// `pfbank` still holds the OLD bit.  `badlands_pf` samples `pfbank_d0` AT
	// `pfbank_wr`, so the pulse is delayed one clk_sys and the value is taken
	// from the settled flop.  Wiring them straight through would load the bank
	// one write behind, and the picture would silently use the previous bank.
	logic        pfbank_we_q;
	always_ff @(posedge clk_sys) pfbank_we_q <= pfbank_we;

	// the sound mailbox
	logic        audwr_stb, audrd_stb, audres, audfull, audirq_n;
	logic  [7:0] audwr_data, audrd_data;
	logic        snd_rom_wr;
	logic [15:0] snd_rom_addr;
	logic  [7:0] snd_rom_data;

	// GAL 9N pin 12: the D8 = AUDFULL read at 0xFC0000 lives in the VIDEO block
	// on the real board
	logic        misc2_n, audfull_d8, audfull_d8_oe;

	logic        res_n;
	logic  [7:0] vid_r, vid_g, vid_b;
	logic        vid_hsync_n, vid_vsync_n, vid_csync_n;
	logic  [7:0] pix_index;

	// the 68000 bus trace badlands_main publishes; simulation only
	/* verilator lint_off UNUSEDSIGNAL */
	logic [23:1] cpu_a;
	logic  [2:0] cpu_fc;
	logic        cpu_as, cpu_rw, cpu_uds_n, cpu_lds_n, cpu_dtack;
	logic [15:0] cpu_wdata, cpu_rdata;
	logic        cctr1, cctr2;
	/* verilator lint_on UNUSEDSIGNAL */

	// =====================================================================
	//  steering: MiSTer control -> a signed COUNT delta -> the quadrature
	//  generator inside badlands_inputs
	// =====================================================================
	logic signed [8:0] wheel0_delta, wheel1_delta;
	logic              wheel0_valid, wheel1_valid;

	badlands_steer u_steer0 (
		.clk(clk_sys), .reset(game_reset), .ce_tick(ce_1h),
		.sens_sel(steer_sens), .enc_sens(enc_sens),
		.stick_x(p1_stick_x), .paddle(p1_paddle), .spinner(p1_spinner),
		.mouse_stb(p1_mouse_stb), .mouse_dx(p1_mouse_dx),
		.dig_left(p1_left), .dig_right(p1_right),
		.delta(wheel0_delta), .delta_valid(wheel0_valid));

	badlands_steer u_steer1 (
		.clk(clk_sys), .reset(game_reset), .ce_tick(ce_1h),
		.sens_sel(steer_sens), .enc_sens(enc_sens),
		.stick_x(p2_stick_x), .paddle(p2_paddle), .spinner(p2_spinner),
		.mouse_stb(1'b0), .mouse_dx(9'sd0),
		.dig_left(p2_left), .dig_right(p2_right),
		.delta(wheel1_delta), .delta_valid(wheel1_valid));

	// =====================================================================
	//  MAIN BOARD — schematic sheets 1 and 2, and the memory path
	// =====================================================================
	// `init_reset` is ~pll_locked ONLY (see the header); the OSD reset and the
	// download hold reach the board through `ext_reset`, which re-runs the POR
	// and the eight-VBLANK LS197 hold exactly as a power cycle does.
	badlands_main #(
		.CLK_HZ(57272724),
		.POR_US(POR_US),
		.PEDAL_IDLE_SILENT(PEDAL_IDLE_SILENT),
		.EEPROM_WRITE_CYCLES(EEPROM_WRITE_CYCLES)
	) u_main (
		.clk(clk_sys), .ce_7m(ce_7m), .ce_14m(ce_14m),
		.init_reset(init_reset), .ext_reset(game_reset),

		.h(h), .v(v), .vblank(vblank_raw),

		.ramd(ramd), .ramd_pf_stb(ramd_pf_stb),
		.ramd_mo0_stb(ramd_mo0_stb), .ramd_mo1_stb(ramd_mo1_stb),

		.cram_cs(cram_cs), .cram_a(cram_a), .cram_we(cram_we),
		.cram_din(cram_din), .cram_dout(cram_dout),

		.pfbank(pfbank), .pfbank_we(pfbank_we),

		.audwr_stb(audwr_stb), .audwr_data(audwr_data),
		.audrd_stb(audrd_stb), .audrd_data(audrd_data),
		.audfull_d8(audfull_d8), .audfull_d8_oe(audfull_d8_oe),
		.misc2_n(misc2_n),
		.audirq_n(audirq_n), .audres(audres),
		.snd_rom_wr(snd_rom_wr), .snd_rom_addr(snd_rom_addr),
		.snd_rom_data(snd_rom_data),

		.pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack), .pf_data(pf_data),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),

		.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
		.SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_CKE(SDRAM_CKE),
		.SDRAM_nCS(SDRAM_nCS), .SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE),

		.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
		.ioctl_index(ioctl_index), .ioctl_upload(ioctl_upload),
		.ioctl_wait(ioctl_wait), .ioctl_upload_req(ioctl_upload_req),
		.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din),

		// Every switch bit is pulled up and grounded when closed, so the board
		// sees ACTIVE LOW.  The OSD gives "pressed".
		.test_n(~service), .p1_fire(p1_fire), .p2_fire(p2_fire),
		.sw1_n(~sw1),
		.wheel0_delta(wheel0_delta), .wheel0_valid(wheel0_valid),
		.wheel1_delta(wheel1_delta), .wheel1_valid(wheel1_valid),
		.pedal0(p1_pedal), .pedal1(p2_pedal), .pedal_invert(pedal_invert),

		.res_n(res_n), .rom_loaded(rom_loaded),
		.cpu_a(cpu_a), .cpu_fc(cpu_fc), .cpu_as(cpu_as), .cpu_rw(cpu_rw),
		.cpu_uds_n(cpu_uds_n), .cpu_lds_n(cpu_lds_n),
		.cpu_wdata(cpu_wdata), .cpu_rdata(cpu_rdata), .cpu_dtack(cpu_dtack)
	);

	// =====================================================================
	//  VIDEO — sheets 1, 3, 4 and 6
	// =====================================================================
	badlands_video #(.V_TOTAL(V_TOTAL)) u_video (
		.clk(clk_sys), .reset(vid_reset), .ce_14m(ce_14m), .ce_7m(ce_7m),

		.ramd(ramd), .ramd_pf_stb(ramd_pf_stb),
		.ramd_mo0_stb(ramd_mo0_stb), .ramd_mo1_stb(ramd_mo1_stb),

		.pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack), .pf_data(pf_data),
		.mo_req(mo_req), .mo_addr(mo_addr), .mo_ack(mo_ack), .mo_data(mo_data),

		.cram_cs(cram_cs), .cram_a(cram_a), .cram_we(cram_we),
		.cram_din(cram_din), .cram_dout(cram_dout),

		.pfbank_wr(pfbank_we_q), .pfbank_d0(pfbank),

		// GAL 9N pin 12: /MISC2 and R/W come from the main board, AUDFULL from
		// the 15J flag on sheet 5, and the D8 pair goes back into
		// badlands_main's read mux.
		.misc2_n(misc2_n), .rw(cpu_rw), .audfull(audfull),
		.audfull_d8(audfull_d8), .audfull_d8_oe(audfull_d8_oe),

		.hblank(hblank), .vblank(vblank),
		.hsync_n(vid_hsync_n), .vsync_n(vid_vsync_n), .csync_n(vid_csync_n),
		.red(vid_r), .green(vid_g), .blue(vid_b), .pix_index(pix_index),

		.h(h), .v(v), .v32(v32), .vblank_raw(vblank_raw),
		.virq_edge(virq_edge), .gfx_late(gfx_late)
	);

	// =====================================================================
	//  SOUND — sheet 5 (there is no separate audio PCB)
	// =====================================================================
	// `audres_n` is the /AUDRES net:
	//     /AUDRES = /RES AND /AUDRESD          (19J LS08)
	// with /AUDRESD low only on a WRITE to 0xFC0000.  `audres` is that write,
	// ACTIVE HIGH, and `res_n` is the 28A LS197 output.  Nothing else on this
	// board resets the sound section — not the watchdog on its own, and not a
	// 68000 RESET instruction.
	badlands_sound u_sound (
		.clk(clk_sys), .ce_2h(ce_2h), .ce_1h(ce_1h), .ce_14m(ce_14m),
		.audres_n(res_n & ~audres),

		.vblank(vblank_raw), .v32(v32),

		.test_n(~service), .coin1(coin1), .coin2(coin2),

		.audwr_stb(audwr_stb), .audwr_data(audwr_data),
		.audrd_stb(audrd_stb), .audrd_data(audrd_data),
		.audfull(audfull), .audirq_n(audirq_n),

		.cctr1(cctr1), .cctr2(cctr2),

		.rom_wr(snd_rom_wr), .rom_addr(snd_rom_addr), .rom_data(snd_rom_data),

		.audio_l(aud_l), .audio_r(aud_r)
	);

	// =====================================================================
	//  outputs
	// =====================================================================
	// badlands_video registers red/green/blue/hblank/vblank/sync together at
	// ce_7m, so ce_pix = ce_7m; arcade_video wants ACTIVE-HIGH sync and the
	// board emits the active-low /HSYNC and /VSYNC.
	assign ce_pix = ce_7m;
	assign vga_r  = vid_r;
	assign vga_g  = vid_g;
	assign vga_b  = vid_b;
	assign hsync  = ~vid_hsync_n;
	assign vsync  = ~vid_vsync_n;

	// =====================================================================
	//  nets that exist because the board has them, with no consumer here
	// =====================================================================
	//  csync_n     the JAMMA-P / AVID-9 composite sync.  MiSTer's arcade_video
	//              takes SEPARATE H and V sync, so the board's own /SYNC is
	//              exported by badlands_video and terminates here.
	//  virq_edge   badlands_main contains the 22J VBLANK latch itself and takes
	//              the RAW SOS-2 pin, as the board does; this pre-edged copy is
	//              the video block's spare.
	//  pix_index   the palette index behind each pixel; simulation only.
	//  gfx_late    a graphics fetch missed its window: a simulation-only
	//              assertion point, never a run-time condition on hardware.
	//  cctr1/2     the two coin-counter solenoid drivers (10E bits 4 and 5).
	//  cpu_*       badlands_main's bus trace; simulation only.
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused_core = &{1'b0,
		vid_csync_n, virq_edge, pix_index, gfx_late,
		cctr1, cctr2,
		cpu_a, cpu_fc, cpu_as, cpu_uds_n, cpu_lds_n, cpu_dtack,
		cpu_wdata, cpu_rdata,
		1'b0};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
