`timescale 1ns/10ps
//============================================================================
//  Bad Lands core PLL -- Altera PLL (Cyclone V fPLL), hand-parameterised.
//
//  Adapted from the Toobin' core's rtl/pll/pll_0002.v (same megafunction
//  instantiation, different counters) rather than re-running the MegaWizard, so
//  the parameters are reviewable in a diff.
//
//  refclk = CLK_50M (FPGA_CLK1_50, 50.000000 MHz)
//
//  outclk_0 = clk_sys = 57.272727 MHz = 50 x 63 / 5 / 11
//      M = 63, N = 5 -> VCO = 630 MHz; C = 11 -> 57.272727 MHz.
//      The target is 4 x the board's 14.318181 MHz crystal = 57.272724 MHz; the
//      error is 0.06 ppm.  Every rate on the board is an integer division:
//        /4 = 14.318181 MHz (SOS-2)    /8 = 7.159091 MHz (pixel clock + 68000)
//        /16 = 3.579545 MHz (YM2151)   /32 = 1.789772 MHz (6502)
//
//  outclk_1 = SDRAM_CLK = the same frequency, shifted +13095 ps (270 deg).
//      The fPLL quantises the shift to VCO/8 = 198.41 ps steps (88 taps over one
//      period) and Quartus rejects any other value, and negative values.
//      13095 ps is tap 66: the centre of the 225-315 deg window that a hardware
//      sweep found to read correctly at this clock frequency.  If the SDRAM
//      controller's timing is ever changed, re-sweep this on hardware -- static
//      timing analysis does not predict it.
//============================================================================
module  pll_0002(

	// interface 'refclk'
	input wire refclk,

	// interface 'reset'
	input wire rst,

	// interface 'outclk0'  -- clk_sys, 57.272727 MHz
	output wire outclk_0,

	// interface 'outclk1'  -- SDRAM_CLK, 57.272727 MHz, +13095 ps (270 deg)
	output wire outclk_1,

	// interface 'locked'
	output wire locked
);

	altera_pll #(
		.fractional_vco_multiplier("false"),
		.reference_clock_frequency("50.0 MHz"),
		.operation_mode("direct"),
		.number_of_clocks(2),
		.output_clock_frequency0("57.272727 MHz"),
		.phase_shift0("0 ps"),
		.duty_cycle0(50),
		.output_clock_frequency1("57.272727 MHz"),
		.phase_shift1("13095 ps"),
		.duty_cycle1(50),
		.output_clock_frequency2("0 MHz"),
		.phase_shift2("0 ps"),
		.duty_cycle2(50),
		.output_clock_frequency3("0 MHz"),
		.phase_shift3("0 ps"),
		.duty_cycle3(50),
		.output_clock_frequency4("0 MHz"),
		.phase_shift4("0 ps"),
		.duty_cycle4(50),
		.output_clock_frequency5("0 MHz"),
		.phase_shift5("0 ps"),
		.duty_cycle5(50),
		.output_clock_frequency6("0 MHz"),
		.phase_shift6("0 ps"),
		.duty_cycle6(50),
		.output_clock_frequency7("0 MHz"),
		.phase_shift7("0 ps"),
		.duty_cycle7(50),
		.output_clock_frequency8("0 MHz"),
		.phase_shift8("0 ps"),
		.duty_cycle8(50),
		.output_clock_frequency9("0 MHz"),
		.phase_shift9("0 ps"),
		.duty_cycle9(50),
		.output_clock_frequency10("0 MHz"),
		.phase_shift10("0 ps"),
		.duty_cycle10(50),
		.output_clock_frequency11("0 MHz"),
		.phase_shift11("0 ps"),
		.duty_cycle11(50),
		.output_clock_frequency12("0 MHz"),
		.phase_shift12("0 ps"),
		.duty_cycle12(50),
		.output_clock_frequency13("0 MHz"),
		.phase_shift13("0 ps"),
		.duty_cycle13(50),
		.output_clock_frequency14("0 MHz"),
		.phase_shift14("0 ps"),
		.duty_cycle14(50),
		.output_clock_frequency15("0 MHz"),
		.phase_shift15("0 ps"),
		.duty_cycle15(50),
		.output_clock_frequency16("0 MHz"),
		.phase_shift16("0 ps"),
		.duty_cycle16(50),
		.output_clock_frequency17("0 MHz"),
		.phase_shift17("0 ps"),
		.duty_cycle17(50),
		.pll_type("General"),
		.pll_subtype("General")
	) altera_pll_i (
		.rst	(rst),
		.outclk	({outclk_1, outclk_0}),
		.locked	(locked),
		.fboutclk	( ),
		.fbclk	(1'b0),
		.refclk	(refclk)
	);
endmodule
