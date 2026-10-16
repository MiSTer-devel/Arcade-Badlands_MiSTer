//============================================================================
//
//  Bad Lands (Atari Games, 1989) MiSTer FPGA core — top-level glue (emu).
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE and NOTICE.md.  Structure inherited from the Vindicators,
//  Xybots and Blasteroids MiSTer cores (GPL-3.0, same author); the steering
//  OSD page follows the Atari System 2 core's (GPL-2.0-or-later, RetroShrimp).
//
//  Wires badlands_core (game logic, one clk_sys domain + integer clock enables)
//  to the MiSTer framework: the PLL, hps_io (ROM download, NVRAM save/restore
//  and controls), the analog alignment stage, arcade_video, audio and the SDRAM
//  chip pins.  Bad Lands runs on a horizontal monitor (MAME ROT0), so there is
//  no rotation.
//
//  Clocking: clk_sys = 57.272724 MHz = 4 x the 14.318181 MHz crystal.  ce_7m =
//  clk_sys/8 is both the pixel rate and the 68000 clock; ce_1h = /16 (YM2151,
//  LETA) and ce_2h = /32 (6502) follow, because on this board the sound section
//  is clocked from the H counter itself.
//
//  ---- the control panel, and why it is shaped like this --------------------
//  * Each player's two Start/Fire buttons are tied together on the board
//    (JAM-17 <-> JAM-22 and JAM-U <-> JAM-Z), so they are one logical input and
//    there is no separate Start switch on this cabinet at all — the manual says
//    "press either of the right buttons".  Both OSD buttons and the pad's Start
//    therefore OR into the same net.
//  * SW1 is a four-position DIP — Freeze, Freeze-Step, Old Gas Pedals and Old
//    Steering Wheels — which the manual says must never be on in normal
//    operation, so all four default off behind their own OSD page carrying that
//    warning: they are development/legacy options, not difficulty settings.
//  * The wheels are optical quadrature encoders, so every MiSTer control
//    (analog stick, D-pad, paddle, spinner, mouse X) is summed into the
//    quadrature generator rather than written into a counter, with no source to
//    choose; see rtl/badlands_steer.sv.
//  * The gas pedal is an LM556 astable whose frequency the pedal changes, so it
//    is a plain button here and the oscillator model lives in badlands_inputs.
//
//============================================================================

`timescale 1ns/1ps

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1        = 0;
assign VGA_SCALER    = 0;
assign VGA_DISABLE   = 0;
assign HDMI_FREEZE   = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT= 0;
assign FB_FORCE_BLANK= 0;

assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

//////////////////////////////////////////////////////////////////

// Horizontal cabinet: the native raster is 336 x 240 on a 4:3 monitor.
wire  [1:0] ar  = status[122:121];
wire [11:0] arx = (ar == 2'd0) ? 12'd4 : 12'(ar - 1'd1);
wire [11:0] ary = (ar == 2'd0) ? 12'd3 : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"Badlands;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[4],Orientation,Original,Flip;",
	"O[22:20],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	"O[3:1],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	// CRT Adjust -- rmonic79's MiSTer-CRT-Adjust engine, core-side variant
	// (rtl/video/badlands_analog_adjust.sv around rtl/video/crt_adjust.sv).  An
	// On/Off master and three amounts; Off is a bit-exact bypass.  H-Position
	// moves the CONTENT inside a native HSync (HPOS_MODE = CONTENTSHIFT), so the
	// old separate "Analog VGA H-Shift" is gone, and V-Shift is the engine's own.
	// Fresh status bits (51..69): the retired page's 23..46 are left unused so a
	// saved config from an older build cannot switch the stage on.  The amounts
	// are hidden whenever the stage is not actually live (see `crt_enable`).
	"P1,CRT Adjust;",
	"P1-;",
	"P1O[51],CRT Adjust,Off,On;",
	"H1P1O[56:52],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[63:57],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,+49,+50,+51,+52,+53,+54,+55,+56,+57,+58,+59,+60,+61,+62,+63,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[69:64],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	// The steering wheels are optical quadrature encoders.  Every MiSTer control
	// turns the disc at once -- analog stick, D-pad, paddle, spinner and (P1)
	// mouse X, summed, with no source to choose -- because a USB spinner often
	// reaches the core as a mouse (rtl/badlands_steer.sv).  The stick rate
	// scales the stick only; the spinner sensitivity scales the spinner, paddle
	// and mouse, in Blasteroids' eighth steps and order.
	"P2,Steering and pedals;",
	"P2-;",
	"P2O[50:47],Spinner sensitivity,1x,1.125x,1.25x,1.375x,1.5x,1.75x,2x,2.5x,3x,4x,0.25x,0.375x,0.5x,0.625x,0.75x,0.875x;",
	"P2O[10:9],Analog stick rate,Normal,High,Low,Lowest;",
	"P2-;",
	// The pedal channels count DOWN: the program ONES-COMPLEMENTS the 8-bit
	// pedal delta before using it (`eori.w #$FF,D1` at 0x008472 / 0x0084A2),
	// which the wheel channels do not, so an up-counting pedal would give FULL
	// throttle at rest and none when pressed.  "Inverted" restores the
	// up-counting pairing, for re-testing only.
	"P2O[17],Gas pedal sense,Normal,Inverted;",
	// SW1: the manual says all four must be OFF in normal operation; they are
	// development / legacy-hardware options, not settings.  The program only
	// checks them at boot: any position ON prints a warning and beeps until it
	// is turned off (ROM 0x008554).
	"P3,Option switch SW1;",
	"P3-;",
	"P3-,All four OFF is normal;",
	"P3-,operation - see the manual.;",
	"P3O[11],SW1-1 Freeze,Off,On;",
	"P3O[12],SW1-2 Freeze-Step,Off,On;",
	"P3O[13],SW1-3 Old gas pedals,Off,On;",
	"P3O[14],SW1-4 Old steering wheels,Off,On;",
	"-;",
	"O[6],Service,Off,On;",
	"-;",
	"T[0],Reset;",
	// The board ties each player's two Start/Fire buttons together, so both of
	// these and the pad's Start reach ONE input.
	"J1,Fire/Start,Fire/Start (2nd btn),Gas,Start,Coin;",
	"jn,A,B,R,Start,Select;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire        direct_video;
wire [21:0] gamma_bus;

// CRT Adjust master enable.  Declared here because status_menumask below reads
// it.  The stage is a 15 kHz adjustment and is forced Off whenever the
// scandoubler runs, for any Scandoubler Fx as well as forced_scandoubler: while
// On, the engine's pixel enable is the uneven 8/9-clock read tick and its
// hb_out folds VBLANK back into HBLANK, which strips the doubled output of its
// VSync (see the hb_in note in badlands_analog_adjust).
wire sd_active  = (status[3:1] != 3'd0) | forced_scandoubler;
wire crt_enable = status[51] & ~sd_active;

wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;
wire        ioctl_upload, ioctl_upload_req;
wire  [7:0] ioctl_upload_index, ioctl_din;

wire [31:0] joystick_0, joystick_1;
wire [15:0] joystick_l_analog_0, joystick_l_analog_1;
wire  [7:0] paddle_0, paddle_1;
wire  [8:0] spinner_0, spinner_1;
wire [24:0] ps2_mouse;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),
	.direct_video(direct_video),

	.buttons(buttons),
	.status(status),
	// H1 = "hide while set": hide the three CRT Adjust amounts whenever the
	// stage is not actually live -- the same `crt_enable` that drives it.
	.status_menumask({14'd0, ~crt_enable, 1'b0}),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	// NVRAM: the 2816A image, ioctl index 2, 2048 bytes -- the
	// <nvram index="2" size="2048"/> element in the MRA.
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_din(ioctl_din),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_l_analog_0(joystick_l_analog_0),
	.joystick_l_analog_1(joystick_l_analog_1),
	.paddle_0(paddle_0),
	.paddle_1(paddle_1),
	.spinner_0(spinner_0),
	.spinner_1(spinner_1),
	.ps2_mouse(ps2_mouse),

	.ps2_key(ps2_key)
);

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;               // 57.272724 MHz
wire clk_sdram;             // same rate, phase-shifted for the SDRAM_CLK pin
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram),
	.locked(pll_locked)
);

// SDRAM clock pin from the phase-shifted PLL output (rtl/pll/pll_0002.v); the
// read-capture phase there is set on real hardware, not from an STA report.
assign SDRAM_CLK = clk_sdram;

// `reset` is the OSD / system reset and reaches the game through ext_reset.
// `~pll_locked` alone is init_reset, which is the ONLY reset badlands_sdram may
// ever see (see badlands_main's u_sdram comment).
wire reset = RESET | status[0] | buttons[1] | ~pll_locked;

///////////////////////   CONTROLS   /////////////////////////////

// MiSTer joystick bits: [0]=Right [1]=Left [2]=Down [3]=Up, then the J1 list:
// [4]=Fire 1 [5]=Fire 2 [6]=Gas [7]=Start [8]=Coin.  The PCB ties one player's
// two Start/Fire buttons together, so all three of these are one input.
wire p1_fire = joystick_0[4] | joystick_0[5] | joystick_0[7];
wire p2_fire = joystick_1[4] | joystick_1[5] | joystick_1[7];

// The gas pedal is a switch that changes the LM556's frequency; the oscillator
// itself is badlands_pedal_556 inside badlands_inputs.
wire p1_pedal = joystick_0[6];
wire p2_pedal = joystick_1[6];

// Coins are read by the SOUND 6502 on /RDIO D1:D0, not by the 68000.
wire coin1 = joystick_0[8];
wire coin2 = joystick_1[8];

wire service = status[6];

// 0 = the DOWN-counting pedal channels the board uses; 1 = the up-counting
// pairing, kept only for A/B re-testing.
wire pedal_invert = status[17];

// SW1 1..4 -> /SWITCHES D3..D0.  "On" in the OSD = the DIP position CLOSED =
// the bit reads 0; badlands_core inverts.
wire [3:0] sw1 = status[14:11];

// Every control is summed into each wheel (rtl/badlands_steer.sv); the two
// feel settings are shared by both players.
wire [1:0] steer_sens   = status[10:9];    // analog stick rate
wire [3:0] enc_sens     = status[50:47];   // spinner / paddle / mouse, 0 = 1x

///////////////////////   CORE   /////////////////////////////////

wire        ce_pix;
wire  [7:0] core_r, core_g, core_b;
wire        core_hs, core_vs, core_hb, core_vb;
wire signed [15:0] aud_l, aud_r;

badlands_core u_core
(
	.clk_sys(clk_sys), .reset(reset), .init_reset(~pll_locked),

	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout), .ioctl_index(ioctl_index), .ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload), .ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din),

	.service(service), .sw1(sw1),
	.p1_fire(p1_fire), .p2_fire(p2_fire),
	.coin1(coin1), .coin2(coin2),
	.p1_pedal(p1_pedal), .p2_pedal(p2_pedal), .pedal_invert(pedal_invert),

	.steer_sens(steer_sens), .enc_sens(enc_sens),
	.p1_stick_x(joystick_l_analog_0[7:0]),
	.p2_stick_x(joystick_l_analog_1[7:0]),
	.p1_paddle(paddle_0), .p2_paddle(paddle_1),
	.p1_spinner(spinner_0), .p2_spinner(spinner_1),
	// Many USB spinners enumerate as a mouse; there is one mouse, so P1 only.
	.p1_mouse_stb(ps2_mouse[24]), .p1_mouse_dx({ps2_mouse[4], ps2_mouse[15:8]}),
	.p1_left(joystick_0[1]), .p1_right(joystick_0[0]),
	.p2_left(joystick_1[1]), .p2_right(joystick_1[0]),

	.ce_pix(ce_pix), .vga_r(core_r), .vga_g(core_g), .vga_b(core_b),
	.hsync(core_hs), .vsync(core_vs), .hblank(core_hb), .vblank(core_vb),

	.aud_l(aud_l), .aud_r(aud_r),

	.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH), .SDRAM_CKE(SDRAM_CKE), .SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE)
);

///////////////////////   AUDIO   ////////////////////////////////

// The board is MONO: JAM-10 SPKR+ and JAM-11 AUDIO+ carry one signal, and
// badlands_sound makes aud_l and aud_r identical by construction.
assign AUDIO_S = 1'b1;              // signed samples
assign AUDIO_L = aud_l;
assign AUDIO_R = aud_r;

///////////////////////   VIDEO   ////////////////////////////////

wire [7:0] adj_r, adj_g, adj_b;
wire       adj_hs, adj_vs, adj_hb, adj_vb, adj_ce;

// CRT Adjust (H-Size / H-Position / V-Shift), rmonic79's crt_adjust engine
// with this core's read-rate and blanking glue.  V_TOTAL follows
// badlands_video's parameter, which defaults to 262.  `crt_enable` is declared
// up with the hps_io wires because status_menumask reads it.
badlands_analog_adjust #(.H_TOTAL(456), .V_TOTAL(262), .CLK_PER_PIX(8)) u_analog_adjust
(
	.clk        (clk_sys),
	.ce_pix     (ce_pix),
	.osd_on     (crt_enable),
	.osd_hsize  (status[56:52]),
	.osd_hpos   (status[63:57]),
	.osd_vshift (status[69:64]),
	.r_in(core_r), .g_in(core_g), .b_in(core_b),
	.hs_in(core_hs), .vs_in(core_vs), .hb_in(core_hb), .vb_in(core_vb),
	.r_out(adj_r), .g_out(adj_g), .b_out(adj_b),
	.hs_out(adj_hs), .vs_out(adj_vs), .hb_out(adj_hb), .vb_out(adj_vb),
	.ce_out(adj_ce)
);

// Scandoubler Fx: 0 None, 1 HQ2x, 2..4 CRT 25/50/75% (arcade_video decodes it;
// any non-zero value also turns the scandoubler on).
wire [2:0] fx = status[3:1];

wire       rotate_ccw = 1'b0;
wire       no_rotate  = 1'b1;
wire       flip       = status[4] & ~direct_video;
wire       video_rotated;

wire vga_de_raw;

// 24-bit RGB, 336 x 240 active.  ce_pix = ce_7m: the board draws ONE pixel per
// H count.  WIDTH is the scandoubler / HQ2x line buffer length, NOT the active
// width: Hq2x's write address counts every input pixel from one HBlank fall to
// the next, i.e. the whole 456-count line, so 456 = H_TOTAL keeps every write
// inside the buffer.  (CRT Adjust never reaches the doubler; it is forced Off
// above.)
arcade_video #(.WIDTH(456), .DW(24)) arcade_video
(
	.clk_video (clk_sys),
	.ce_pix    (adj_ce),
	.RGB_in    ({adj_r, adj_g, adj_b}),
	.HBlank    (adj_hb),
	.VBlank    (adj_vb),
	.HSync     (adj_hs),
	.VSync     (adj_vs),

	.CLK_VIDEO (CLK_VIDEO),
	.CE_PIXEL  (CE_PIXEL),
	.VGA_R     (VGA_R),
	.VGA_G     (VGA_G),
	.VGA_B     (VGA_B),
	.VGA_HS    (VGA_HS),
	.VGA_VS    (VGA_VS),
	.VGA_DE    (vga_de_raw),
	.VGA_SL    (VGA_SL),

	.fx                 (fx),
	.forced_scandoubler (forced_scandoubler),
	.gamma_bus          (gamma_bus)
);

// video_freak SCALE encoding: 0 normal, 1 V-integer, 2 HV-Integer-, 4 HV-Integer.
wire [2:0] scale_sel = (status[22:20] == 3'd0) ? 3'd0 :
                       (status[22:20] == 3'd1) ? 3'd1 :
                       (status[22:20] == 3'd2) ? 3'd4 :
                                                 3'd2;

video_freak video_freak
(
	.CLK_VIDEO  (CLK_VIDEO),
	.CE_PIXEL   (CE_PIXEL),
	.VGA_VS     (VGA_VS),
	.HDMI_WIDTH (HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE     (VGA_DE),
	.VIDEO_ARX  (VIDEO_ARX),
	.VIDEO_ARY  (VIDEO_ARY),
	.VGA_DE_IN  (vga_de_raw),
	.ARX        (arx),
	.ARY        (ary),
	.CROP_SIZE  (12'd0),
	.CROP_OFF   (5'd0),
	.SCALE      (scale_sel)
);

screen_rotate screen_rotate (.*);

///////////////////////   STATUS LED   ///////////////////////////

assign LED_USER = ioctl_download;

endmodule
