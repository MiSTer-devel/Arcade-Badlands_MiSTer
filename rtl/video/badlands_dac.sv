`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the R2R video DACs (RN1/RN2/RN3, Q5/Q6/Q7, the 2J/4J F260
//  zero-detect "black pull" and the 6F 7406 open-collector sinks).
//  Schematic sheet 6.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  Green is six bits, red and blue are five.  Each channel is an R2R ladder
//  into a 2N3904 emitter follower; the ladder's D0 (pin 2) is tied to GND on
//  red and on blue, and carries GREEN0 on green.  MAME uses
//  `palette_device::IRGB_1555`, in which colour-word bit 15 adds an LSB to all
//  three guns — the board wires that bit to the green ladder alone, so do not
//  implement IRGB_1555.  Expect a small systematic colour difference from MAME
//  on any entry with bit 15 set.
//
//  The colour word feeds the three guns as:
//      byte 0 = { GREEN0, RED5, RED4, RED3, RED2, RED1, GREEN5, GREEN4 }
//      byte 1 = { GREEN3, GREEN2, GREEN1, BLUE5, BLUE4, BLUE3, BLUE2, BLUE1 }
//
//  The analogue transfer is linear in the code above zero, with a single
//  non-linear "blacker than black" point at Cn5:Cn1 = 0 produced by the F260
//  zero-detect sinking the ladder node through 1.5 K.  That point is an
//  artefact of the analogue stage and maps to 0 in a digital pipeline, so the
//  default output is bit replication — exact at both endpoints, maximum error
//  1 LSB:
//
//      red8   = { r5, r4, r3, r2, r1, r5, r4, r3 }
//      green8 = { g5, g4, g3, g2, g1, g0, g5, g4 }
//      blue8  = { b5, b4, b3, b2, b1, b5, b4, b3 }
//
//  `VOLTAGE_TABLE = 1` selects the alternative mapping built from the derived
//  ladder voltages instead — 0.0922 V per step, a 0.0461 V GREEN0 half-step,
//  0.335 V at code 0 (0.367 V with GREEN0), 3.501 V at code 31, normalised so
//  code 0 -> 0 and code 31 -> 255.  It is not what the core ships; it is here
//  because anyone reproducing the analogue stage needs the numbers.  VOH, Vbe,
//  beta and VOL(7406) are each inferred to about +-10 %.
//============================================================================

module badlands_dac #(
	// 0 = bit replication (default, what the core ships)
	// 1 = the analogue voltage table, normalised to 0..255
	parameter bit VOLTAGE_TABLE = 1'b0
)(
	input  logic [7:0] byte0,     // 1K HC273
	input  logic [7:0] byte1,     // 5K HC273
	output logic [7:0] red,
	output logic [7:0] green,
	output logic [7:0] blue,
	// the raw gun codes, for anyone reproducing sheet 6
	output logic [4:0] red5,
	output logic [5:0] green6,
	output logic [4:0] blue5
);

	// ---- the colour-word bit map ------------------------------------------
	assign red5   =  byte0[6:2];                              // RED5..RED1
	assign green6 = {byte0[1:0], byte1[7:5], byte0[7]};       // GREEN5..GREEN0
	assign blue5  =  byte1[4:0];                              // BLUE5..BLUE1

	// ---- the voltage table, in units of 0.1 mV ----------------------------
	// V(0) = 3350 (+320 with GREEN0);  V(n>=1) = 35010 - 922*(31-n) (+461 with
	// GREEN0).  Normalised: out = round(255 * (V - 3350) / 31660).
	function automatic logic [7:0] vtab(input logic [4:0] n, input logic g0);
		int volts, num;
		begin
			if (n == 5'd0) volts = 3350 + (g0 ? 320 : 0);
			else           volts = 35010 - 922 * (31 - int'(n)) + (g0 ? 461 : 0);
			num = (volts - 3350) * 255 + 15830;  // + half a denominator, to round
			if (num < 0) num = 0;
			volts = num / 31660;
			if (volts > 255) volts = 255;
			vtab = 8'(volts);
		end
	endfunction

	wire [4:0] g_hi = green6[5:1];
	wire       g_lo = green6[0];

	wire [7:0] red_v   = vtab(red5,  1'b0);
	wire [7:0] green_v = vtab(g_hi,  g_lo);
	wire [7:0] blue_v  = vtab(blue5, 1'b0);

	// bit replication, exact at both endpoints
	wire [7:0] red_b   = { red5,   red5[4:2]   };
	wire [7:0] green_b = { green6, green6[5:4] };
	wire [7:0] blue_b  = { blue5,  blue5[4:2]  };

	assign red   = VOLTAGE_TABLE ? red_v   : red_b;
	assign green = VOLTAGE_TABLE ? green_v : green_b;
	assign blue  = VOLTAGE_TABLE ? blue_v  : blue_b;

endmodule
