`timescale 1ns/1ps
//============================================================================
//  Bad Lands — VMATCH 17U (Atari 137552-001), the motion-object vertical
//  matcher and stamp-row address generator.  Schematic sheet 4 grid 17U.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//  Structural reference: the Xybots core's `xybots_mo_vmatch.sv` and the
//  Blasteroids core's sibling custom, re-derived here for the 16 x 8 stamp
//  geometry.
//
//  Wiring:
//      D15:D0   = RAMD15:0            INCLK  = PVCLK        OUTCLK = 8HD6H
//      VCLK     = /256H (one edge per line, at H = 0)        VRES = /VRES
//      OE       = GND (always enabled)
//      VFLIPEN  = GND      HFLD = no connect
//          -> Bad Lands motion objects cannot flip, matching MAME's
//             hflipmask = 0 / vflipmask = 0.
//      RA14:RA2 -> the MO stamp ROMs' A15:A3
//      VMATCHD (inverted pin) -> 18S/2 D
//
//  The internals are inferred: this is the only algorithm consistent with the
//  pin widths, the MO RAM layout, the ROM geometry and MAME.
//
//      Y      = MOV[15:7]                 (9 bits)
//      height = MOV[3:0] + 1              (tiles, 8 scanlines each)
//      code   = MOP[11:0]
//
//      D      = the line being prepared = V + 1  (the buffer written while the
//               beam is on line V is displayed on line V+1), wrapping at V_TOTAL
//      U      = (Y + D) mod 512
//      vsum   = U[8:3] + MOV[3:0] + 1     (7 bits, 0..79)
//      match  = vsum[6]                   i.e. U >= 512 - 8*height
//      row    = vsum[3:0]                 tile row inside the object, 0..15
//      line   = U[2:0]                    pixel row inside the 8-row stamp
//
//      RA14:5 = (code[9:0] + row) mod 1024
//      RA4:2  = line
//      VMATCHD = NOT match                (the pin carries an inversion bubble)
//
//  This reproduces MAME's atarimo arithmetic exactly: MAME draws the sprite
//  from `ypos = (-Y - 8*height) mod 512` downwards, and solving
//  `sy = ypos + 8*t + line` for the scanline gives the same D.
//
//  The row carry: `RA14:5` is ten bits and is the only field VMATCH adds `row`
//  into.  `code[11:10]` reaches the ROM chip select through 13V/15V without the
//  carry, so a tall object whose `code[9:0] + row` crosses 1024 wraps inside
//  its own 1024-stamp device instead of stepping into the next one.  MAME
//  increments the full 12-bit code instead; `code_hi_carry` exports MAME's
//  version of the top two bits so `badlands_mo`'s `MO_CODE_CARRY` parameter can
//  select either behaviour.
//
//  The D counter is modelled literally from /VRES and VCLK: /VRES is low over
//  the last line of the frame and VCLK (= /256H) rises once per line at H = 0,
//  so loading 0 on the reset line and incrementing otherwise gives D = V + 1
//  for V = 0..V_TOTAL-2 and D = 0 on the last line.
//============================================================================

module badlands_vmatch (
	input  logic        clk,
	input  logic        reset,

	input  logic [15:0] ramd,          // D15:D0

	// ---- the four clocks, as ce_7m-aligned edge markers -------------------
	input  logic        inclk_rise,    // PVCLK 0->1 at n = 5->6 : take MOP
	input  logic        inclk_fall,    // PVCLK 1->0 at n = 7->8 : take MOV
	input  logic        outclk_rise,   // 8HD6H 0->1 at n = 13->14: issue RA
	input  logic        line_start,    // VCLK rising, H = 0
	input  logic        vres_n,        // /VRES, low over the last line

	// ---- outputs, registered at OUTCLK ------------------------------------
	output logic [12:0] ra,            // RA14:RA5 then RA4:RA2
	output logic [1:0]  code_hi_carry, // (code + row)[11:10] — the MAME leg
	output logic        vmatchd_n,     // active low = "this entry matches"

	// ---- the same values combinationally, for the byte-0 fetch that is
	//      issued on the very edge that registers them -------------------
	output logic [12:0] ra_next,
	output logic [1:0]  code_hi_carry_next,
	output logic        vmatchd_n_next,

	// ---- the line being prepared ------------------------------------------
	output logic [8:0]  dline
);

	// =====================================================================
	// 1. The two input captures
	// =====================================================================
	logic [11:0] code_r;      // MOP, complete on RAMD at n = 5
	logic [8:0]  y_r;         // MOV D15:7, complete on RAMD at n = 7
	logic [3:0]  hgt_r;       // MOV D3:0 = height - 1

	always_ff @(posedge clk) begin
		if (reset) begin
			code_r <= 12'd0;
			y_r    <= 9'd0;
			hgt_r  <= 4'd0;
		end else begin
			if (inclk_rise) code_r <= ramd[11:0];
			if (inclk_fall) begin
				y_r   <= ramd[15:7];
				hgt_r <= ramd[3:0];
			end
		end
	end

	// =====================================================================
	// 2. The line counter D (VCLK / /VRES)
	// =====================================================================
	always_ff @(posedge clk) begin
		if (reset)            dline <= 9'd0;
		else if (line_start)  dline <= vres_n ? (dline + 9'd1) : 9'd0;
	end

	// =====================================================================
	// 3. The match arithmetic — combinational
	// =====================================================================
	wire [8:0]  u    = y_r + dline;                     // mod 512 by width
	wire [6:0]  vsum = {1'b0, u[8:3]} + {3'b000, hgt_r} + 7'd1;
	wire        match = vsum[6];
	wire [3:0]  row   = vsum[3:0];
	wire [2:0]  lin   = u[2:0];
	// vsum[5:4] carry no information: `match` is bit 6 and `row` is bits 3:0.
	// verilator lint_off UNUSEDSIGNAL
	wire        unused_vsum = &{1'b0, vsum[5:4]};
	// verilator lint_on UNUSEDSIGNAL

	// The 10-bit carry stop lives here: only code[9:0] is added.
	wire [9:0]  ra_hi = code_r[9:0] + {6'd0, row};
	// ...and this is what MAME would produce instead.  Only the top two bits are
	// wanted: the low ten are identical to `ra_hi` by construction.
	wire [11:0] code_sum = code_r + {8'd0, row};
	// verilator lint_off UNUSEDSIGNAL
	wire        unused_cs = &{1'b0, code_sum[9:0]};
	// verilator lint_on UNUSEDSIGNAL

	assign ra_next            = { ra_hi, lin };
	assign code_hi_carry_next = code_sum[11:10];
	assign vmatchd_n_next     = ~match;

	// =====================================================================
	// 4. The OUTCLK register (8HD6H rising, n = 13 -> 14)
	// =====================================================================
	always_ff @(posedge clk) begin
		if (reset) begin
			ra            <= 13'd0;
			code_hi_carry <= 2'd0;
			vmatchd_n     <= 1'b1;
		end else if (outclk_rise) begin
			ra            <= ra_next;
			code_hi_carry <= code_hi_carry_next;
			vmatchd_n     <= vmatchd_n_next;
		end
	end

endmodule
