`timescale 1ns/1ps
//============================================================================
//  Bad Lands bring-up test pattern.  Nothing here models hardware; it is
//  compiled in only when BADLANDS_TESTPAT is defined.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//  Adapted from the Vindicators core's `rtl/video/vind_testpat.sv` (GPL-3.0,
//  same author), re-cut for the 336 x 240 raster and widened to 8 bits per
//  channel.
//
//  Its job is to show, on a real MiSTer, that
//    * the PLL locks and clk_sys / ce_7m actually toggle (the box moves),
//    * the 456 x 262 / 336 x 240 raster from badlands_sos2_sync locks a monitor
//      and MiSTer's scaler — the 2-pixel white frame sits exactly on the
//      HBlank/VBlank edges, so a mis-placed blanking edge shows up as a clipped
//      or floating border,
//    * the RGB path through badlands_analog_adjust and arcade_video is wired
//      right (bars in the documented order, the grey ramp monotonic L->R).
//
//  336 = 8 bars x 42 px = 16 ramp steps x 21 px, both exact.
//============================================================================

module badlands_testpat (
	input  logic       clk,
	input  logic       reset,
	input  logic       ce,            // ce_7m, one pulse per pixel
	input  logic [8:0] h_count,       // H count from the sync generator
	input  logic [8:0] v_count,       // V line  from the sync generator
	input  logic       hblank,
	input  logic       vblank,
	input  logic       frame_start,

	output logic [7:0] r,
	output logic [7:0] g,
	output logic [7:0] b
);
	localparam logic [8:0] H_ACTIVE = 9'd336;
	localparam logic [8:0] V_ACTIVE = 9'd240;

	localparam logic [8:0] BOX_SIZE = 9'd32;
	localparam logic [8:0] BOX_XMAX = H_ACTIVE - BOX_SIZE;   // 304
	localparam logic [8:0] BOX_YMAX = V_ACTIVE - BOX_SIZE;   // 208

	localparam logic [5:0] BAR_W    = 6'd42;
	localparam logic [4:0] STEP_W   = 5'd21;

	// ---- bouncing box, one step per frame --------------------------------
	logic [8:0] box_x, box_y;
	logic       box_dx, box_dy;   // 1 = moving in the increasing direction

	always_ff @(posedge clk) begin
		if (reset) begin
			box_x  <= 9'd40;
			box_y  <= 9'd40;
			box_dx <= 1'b1;
			box_dy <= 1'b1;
		end else if (ce && frame_start) begin
			if (box_dx) begin
				if (box_x >= BOX_XMAX - 9'd2) begin box_x <= BOX_XMAX; box_dx <= 1'b0; end
				else                                box_x <= box_x + 9'd2;
			end else begin
				if (box_x <= 9'd2)            begin box_x <= 9'd0;    box_dx <= 1'b1; end
				else                                box_x <= box_x - 9'd2;
			end
			if (box_dy) begin
				if (box_y >= BOX_YMAX - 9'd1) begin box_y <= BOX_YMAX; box_dy <= 1'b0; end
				else                                box_y <= box_y + 9'd1;
			end else begin
				if (box_y == 9'd0)            begin                    box_dy <= 1'b1; end
				else                                box_y <= box_y - 9'd1;
			end
		end
	end

	// ---- per-line bar / ramp indices -------------------------------------
	// Counters instead of x/42 and x/21 so no divider is inferred.  They are
	// cleared throughout HBLANK, so at h_count == k they read exactly k.
	logic [5:0] bar_cnt;
	logic [2:0] bar_idx;
	logic [4:0] step_cnt;
	logic [3:0] ramp_lvl;

	always_ff @(posedge clk) begin
		if (reset) begin
			bar_cnt  <= 6'd0;
			bar_idx  <= 3'd0;
			step_cnt <= 5'd0;
			ramp_lvl <= 4'd0;
		end else if (ce) begin
			if (hblank) begin
				bar_cnt  <= 6'd0;
				bar_idx  <= 3'd0;
				step_cnt <= 5'd0;
				ramp_lvl <= 4'd0;
			end else begin
				if (bar_cnt == BAR_W - 6'd1) begin
					bar_cnt <= 6'd0;
					bar_idx <= bar_idx + 3'd1;
				end else begin
					bar_cnt <= bar_cnt + 6'd1;
				end
				if (step_cnt == STEP_W - 5'd1) begin
					step_cnt <= 5'd0;
					ramp_lvl <= ramp_lvl + 4'd1;
				end else begin
					step_cnt <= step_cnt + 5'd1;
				end
			end
		end
	end

	// ---- pattern ---------------------------------------------------------
	wire       active    = ~hblank & ~vblank;
	wire [8:0] x         = h_count;
	wire [8:0] y         = v_count;

	// 2-pixel white frame exactly on the 336 x 240 visible edge.
	wire       on_border = active &&
	                       ((x < 9'd2) || (x >= H_ACTIVE - 9'd2) ||
	                        (y < 9'd2) || (y >= V_ACTIVE - 9'd2));

	// 32 x 32 bouncing box.
	wire       in_box    = active &&
	                       (x >= box_x) && (x < box_x + BOX_SIZE) &&
	                       (y >= box_y) && (y < box_y + BOX_SIZE);

	// Bottom 60 lines: 16-step grey ramp.
	wire       in_ramp   = active && (y >= 9'd180);

	logic [7:0] bar_r, bar_g, bar_b;
	always_comb begin
		case (bar_idx)                                          // SMPTE order
			3'd0:    {bar_r, bar_g, bar_b} = {8'hFF, 8'hFF, 8'hFF};  // white
			3'd1:    {bar_r, bar_g, bar_b} = {8'hFF, 8'hFF, 8'h00};  // yellow
			3'd2:    {bar_r, bar_g, bar_b} = {8'h00, 8'hFF, 8'hFF};  // cyan
			3'd3:    {bar_r, bar_g, bar_b} = {8'h00, 8'hFF, 8'h00};  // green
			3'd4:    {bar_r, bar_g, bar_b} = {8'hFF, 8'h00, 8'hFF};  // magenta
			3'd5:    {bar_r, bar_g, bar_b} = {8'hFF, 8'h00, 8'h00};  // red
			3'd6:    {bar_r, bar_g, bar_b} = {8'h00, 8'h00, 8'hFF};  // blue
			default: {bar_r, bar_g, bar_b} = {8'h00, 8'h00, 8'h00};  // black
		endcase
	end

	wire [7:0] ramp8 = {ramp_lvl, ramp_lvl};

	logic [7:0] nr, ng, nb;
	always_comb begin
		if      (!active)   {nr, ng, nb} = {8'h00, 8'h00, 8'h00};
		else if (on_border) {nr, ng, nb} = {8'hFF, 8'hFF, 8'hFF};
		else if (in_box)    {nr, ng, nb} = {8'hFF, 8'h88, 8'h00};   // orange box
		else if (in_ramp)   {nr, ng, nb} = {ramp8, ramp8, ramp8};
		else                {nr, ng, nb} = {bar_r, bar_g, bar_b};
	end

	always_ff @(posedge clk) begin
		if (reset) begin
			r <= 8'h00;
			g <= 8'h00;
			b <= 8'h00;
		end else if (ce) begin
			r <= nr;
			g <= ng;
			b <= nb;
		end
	end
endmodule
