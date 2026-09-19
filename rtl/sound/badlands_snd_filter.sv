`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- a digital model of the sheet-5 analogue
//  chain: the 5E/2E LM324 stages and the TDA2030 that stand between the
//  YM3012's two sample-and-hold outputs and JAM-10/JAM-11.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  The state-variable-filter realisation and the "pipeline the multiplies
//  across the clk_sys cycles between two ce pulses" structure are taken from
//  the Blasteroids MiSTer core's `blstroid_jsa_lpf.sv` (GPL-3.0, same author).
//  The topology, the values and every coefficient are Bad Lands' own: this
//  chain is mono, it has a first-order high-pass the JSA does not, and its
//  low-pass corner is fixed (no /MIX-switched capacitor).
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  The chain:
//   #  stage                                          mid-band gain  corner
//   1  C54/C55 1 nF on each YM3012 output             1              deglitch
//   2  5E LM324 unity buffers                         1              --
//   3  summing inverter 2E(5,6,7): R67 = R68 = 18 K,  -R63/R67 =     1st-order
//      R63 = 47 K, C53 = 0.22 uF -- both channels     -2.611         high-pass
//      weighted equally                               (+8.34 dB)     80.4 Hz
//   4  Sallen-Key unity LP 2E(8,9,10): R62 = R64 =    1              f0 =
//      12 K, C49 = 2.2 nF, C50 = 1 nF                                8.94 kHz
//                                                                    Q = 0.742
//   5  R1 10 K volume pot, wiper into R101 1.5 K      0 .. 1         --
//   6  TDA2030 non-inverting: R60 = 1 K, R61 = 5.1 K  1 + 5.1 = 6.1  HP 7.2 Hz
//                                                     (+15.7 dB)     Zobel
//                                                                    15.9 kHz
//   7  C4 470 uF into an 8 ohm speaker                --             HP 42.3 Hz
//
//  What this file implements is mix = ch1 + ch2 (equal weight, R67 = R68), then
//  stage 3's first-order high-pass and stage 4's second-order low-pass:
//   HP  R67 || R68 = 9 000 ohm with C53 = 0.22 uF
//       fc = 1 / (2*pi*9000*0.22e-6)                   =   80.381 Hz
//   LP  f0 = 1 / (2*pi*R*sqrt(C49*C50))                = 8941.7   Hz
//       Q  = 0.5*sqrt(C49/C50) = 0.5*sqrt(2.2)         =    0.741620
//  A DC block for the 7.2 Hz / 42.3 Hz post-volume poles and a first-order LP
//  for the 15.9 kHz Zobel are deliberately left out: those poles sit an octave
//  and a half below the 80.4 Hz one already here and would move the audible
//  corner by under 0.6 dB at 100 Hz, and the Zobel sits above the 8.94 kHz
//  second-order roll-off where the response is already past -12 dB.
//
//  Output scaling is the one number that is not the schematic's.  The physical
//  mid-band gain from one YM3012 channel to the speaker is 2.611 x 6.1 = 15.93
//  (+24.0 dB), but jt51's `xleft`/`xright` are already full-scale signed
//  16-bit, so applying 15.93 to their sum would need 21 bits.  The chain's
//  absolute gain is set on the board by R1, the volume pot (stage 5), which on
//  MiSTer is the OSD volume.  This module therefore applies the chain's shape
//  exactly and renormalises the gain to a single mid-band scalar of
//  15/32 = 0.46875 from the summing node, chosen so that
//      full-scale ch1 + full-scale ch2 = +/-65536
//        x 1.0042 (the Q = 0.742 magnitude peak at f0)
//        x 15/32                                   = +/-30 849
//  leaves ~6 % headroom in signed 16-bit, and a full-scale step (4.3 % time-
//  domain overshoot) reaches +/-32 040, still short of +/-32 767.  The
//  saturator below therefore never engages for a YM output; it exists so a
//  pathological transient is a quiet clamp instead of a loud wrap.
//
//  Rate and arithmetic.  Clocked by `ce` = ce_2h = 1.7897725 MHz, the same
//  enable the 6502 runs on.  The YM3012 is a 55.9 kHz sample-and-hold, so
//  feeding its zero-order-held output into a filter running 32x faster is what
//  the board does: the Sallen-Key IS the reconstruction filter, and it
//  attenuates the 55.9 kHz image by ~31 dB.
//   * the high-pass is a leaky-integrator DC blocker, `dc += k*(x-dc)`,
//     `y = x - dc`, with k = 2*pi*fc/fs = 2.82186e-4.  Q20: k = 296/2^20
//     => fc = 80.39 Hz (0.01 % high).
//   * the low-pass is a Chamberlin state-variable filter, parameterised
//     directly by the (f, q) the schematic gives:
//         f = 2*sin(pi*f0/fs) = 0.0313896  -> Q16  2057  (f0 = 8941.3 Hz)
//         q = 1/Q             = 1.348401   -> Q16 88369  (Q  = 0.741623)
//     An SVF rather than a direct-form biquad because f0/fs = 0.005:
//     direct-form poles that close to z = 1 need far more coefficient
//     precision, while the SVF stays accurate.  Stability needs f*q < 2 and
//     f < 2; here f*q = 0.0423.
//   * the multiplies are pipelined across the clk_sys cycles between two ce
//     pulses (3 clocks of settling against the 32 available) rather than
//     chained into one cycle, which misses setup in Quartus.  Do not replace
//     the pipeline with a blanket SDC multicycle exception: it would be false
//     about precisely these registers, which move on every clock.
//============================================================================

module badlands_snd_filter
(
	input  logic        clk,
	input  logic        ce,                        // ce_2h = 1.7897725 MHz
	input  logic        bypass,                    // 1 = raw sum (MAME's model)
	input  logic signed [15:0] ch1,                // YM3012 pin 10 -> R68 18 K
	input  logic signed [15:0] ch2,                // YM3012 pin  9 -> R67 18 K
	output logic signed [15:0] out,
	output logic        clipped                    // sticky: the saturator fired
);

	// ---- stage 3a: the summing node, equal weight (R67 = R68 = 18 K) --------
	wire signed [17:0] mix = {{2{ch1[15]}}, ch1} + {{2{ch2[15]}}, ch2};

	// ---- stage 3b: 1st-order high-pass, 80.4 Hz ----------------------------
	// dc is Q20 with 18 integer bits, so k = 296 / 2^20.
	localparam signed [9:0] K_HP = 10'sd296;
	localparam int          HW   = 38;             // 18 integer + 20 fraction

	logic signed [HW-1:0] dc;
	initial dc = '0;
	wire  signed [HW-1:0] x_q20 = {mix, 20'd0};
	wire  signed [HW-1:0] err   = x_q20 - dc;
	wire  signed [HW+9:0] k_err = err * K_HP;

	logic signed [HW-1:0] k_err_r;
	initial k_err_r = '0;
	// `always @(posedge clk)` and not `always_ff`: the pipeline registers have
	// no reset, so their power-up value comes from the `initial` blocks -- and
	// IEEE 1800 9.2.2.4 forbids an always_ff variable being written by another
	// process.  The same applies to every register below.
	always @(posedge clk) k_err_r <= HW'(k_err >>> 20);

	always @(posedge clk) if (ce) dc <= dc + k_err_r;

	// A part-select is unsigned; land it on a signed wire of the same width so
	// the bit pattern survives and the subtraction below is two's complement.
	wire signed [17:0] dc_int = dc[HW-1:20];
	wire signed [17:0] hp_in  = mix - dc_int;

	// ---- stage 4: Sallen-Key low-pass, 8941.7 Hz, Q = 0.7416 ---------------
	localparam signed [18:0] F_LP = 19'sd2057;     // Q16 of 0.0313896
	localparam signed [18:0] Q_LP = 19'sd88369;    // Q16 of 1.348401 (= 1/Q)
	localparam int           SW   = 34;            // 18 integer + 16 fraction

	logic signed [SW-1:0] lp, bp;
	initial begin lp = '0; bp = '0; end
	wire  signed [SW-1:0] in_q16 = {hp_in, 16'd0};

	// low  = low  + f*band
	// high = in   - low - q*band
	// band = band + f*high
	logic signed [SW-1:0] f_bp_r, q_bp_r, hp_r, f_hp_r;
	initial begin f_bp_r = '0; q_bp_r = '0; hp_r = '0; f_hp_r = '0; end

	wire signed [SW+18:0] f_bp = F_LP * bp;
	wire signed [SW+18:0] q_bp = Q_LP * bp;
	always @(posedge clk) begin
		f_bp_r <= SW'(f_bp >>> 16);
		q_bp_r <= SW'(q_bp >>> 16);
	end

	wire signed [SW-1:0] lp_nxt = lp + f_bp_r;
	wire signed [SW-1:0] hp_sv  = in_q16 - lp_nxt - q_bp_r;
	always @(posedge clk) hp_r <= hp_sv;

	wire signed [SW+18:0] f_hp = F_LP * hp_r;
	always @(posedge clk) f_hp_r <= SW'(f_hp >>> 16);

	wire signed [SW-1:0] bp_nxt = bp + f_hp_r;

	always @(posedge clk) if (ce) begin
		lp <= lp_nxt;
		bp <= bp_nxt;
	end

	// ---- stages 3/6: the mid-band scalar, renormalised (see the header) -----
	wire signed [17:0] lp_int = lp[SW-1:16];
	wire signed [22:0] scaled = ($signed({{5{lp_int[17]}}, lp_int}) * 23'sd15) >>> 5;

	function automatic logic signed [15:0] sat16(input logic signed [22:0] v);
		if      (v >  23'sd32767) sat16 =  16'sd32767;
		else if (v < -23'sd32768) sat16 =  16'sh8000;
		else                      sat16 =  v[15:0];
	endfunction

	wire sat_now = (scaled > 23'sd32767) || (scaled < -23'sd32768);

	logic clip_sticky;
	initial clip_sticky = 1'b0;
	always @(posedge clk) if (ce && sat_now && !bypass) clip_sticky <= 1'b1;
	assign clipped = clip_sticky;

	assign out = bypass ? mix[15:0] : sat16(scaled);

	// In bypass the 18-bit sum is truncated to its low 16 bits, which is what
	// MAME does (it routes both YM channels at 0.30 each and applies no filter).
	// Bypass exists only for that comparison and is never used by the core.

endmodule
