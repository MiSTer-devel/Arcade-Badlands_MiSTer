`timescale 1ns/1ps
//============================================================================
//  Bad Lands video / work RAM -- the one 8 K x 8 SRAM at 22V, its four address
//  multiplexers, GAL 21R (136074-1002), the 1H byte-lane scheme and the
//  19N / 22K / 28N / 28L / 22N data path.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  The single-BRAM + clock-enable idiom follows
//  `Arcade-Blasteroids_MiSTer/rtl/mem/blstroid_ram8.sv` (GPL-3.0, same
//  author); everything else here is Bad Lands' own, because Blasteroids has
//  separate playfield and work RAMs and this board has exactly one.
//
//  ---- the part ------------------------------------------------------------
//  One 8 K x 8 100 ns SRAM (137535-004) at 22V.  /CS1 = GND and CS2 = +5 V, so
//  it is permanently selected; /WE = /RAMWP, /OE = /RAMOE, and A0 = 1H always
//  -- that pin is the byte-lane select and nothing else.  A12:A1 come from the
//  four multiplexers.  There is no second port: the CPU and the video side
//  share the RAM by time slot, and that is why the CPU waits.
//
//  ---- the slot schedule ---------------------------------------------------
//  (4H,2H) cuts every group of 8 H counts into four 2-count slots; 1H picks
//  the byte inside a slot.  h = H mod 8:
//
//    h = 0,1  CPU            A12:A1 = 68000 A12:A1                    (/UTIME low)
//    h = 2,3  playfield      A11:A1 = { V[7:3] , H[8:3] }
//    h = 4,5  motion object  A12=1, A8=8H, A7=2H(=0), A6:A2=H[8:4], A1=0
//    h = 6,7  motion object  as above but A7 = 2H (=1)
//
//    PF word  = 0xFFE000 + 2*{V[7:3],H[8:3]}
//    MO word  = 0xFFF000 + 0x80*{8H,2H} + 4*H[8:4]     -- 32 entries, 4 B apart
//
//  ---- GAL 21R (136074-1002), verbatim -------------------------------------
//  GAL16V8 registered.  CLK (pin 1) = 14M, /OE (pin 11) = GND, pin 12 = /RAM.
//  Five registered outputs 13..17 (the sheet's "PAL16R4" is nominal; pin 13's
//  register is unused and unwired).  Pins 18-19 combinatorial.
//
//    RTIME := 1H  & RAM & RTIME                            (hold a)
//           + /2H & RAM & RTIME                            (hold b)
//           + /7M & AS & LDS & 1H & 2H & 4H & RAM          (set, low lane)
//           + /7M & AS & UDS & 1H & 2H & 4H & RAM          (set, high lane)
//           ;  pin 14 = /RTIME
//
//    PIN15 := /(  7M & WR & LDS &  1H & /2H & /4H & RTIME
//              + /7M & WR & LDS & /1H & /2H & /4H & RTIME
//              +  7M & WR & UDS & /1H & /2H & /4H & RTIME
//              + /7M & AS & WR & UDS & 1H & 2H & 4H & RAM )
//
//    UTIME := ( 7M & /2H & /4H) + (/1H & /2H & /4H) + (/7M & 1H & 2H & 4H)
//           ;  pin 16 = /UTIME
//
//    RAMRE := AS & RD & RAM                                ;  pin 17 = /RAMRE
//
//    /RAMWE = AS & WR & RAM & UTIME & (/RAMOE high)        (combinatorial)
//    /RAMOE = (AS & RD & RAM) + (RTIME = 0)                (combinatorial)
//
//  /UTIME is "uP time", not "upper byte": a two-count CPU-slot window covering
//  both byte cycles.  /RAM arrives with no /AS term and is used raw, exactly
//  as pin 12 receives it.
//
//  ---- the sampling phase, which is forced ---------------------------------
//  23R (which makes 7M) and GAL 21R are both clocked by 14M rising, so the GAL
//  samples 7M, 1H, 2H, 4H pre-edge.  With `pcnt` 0..7 the position inside an H
//  count (pcnt == 7 is the ce_7m cycle, 7M high over pcnt 0..3), the two 14M
//  edges land at the starts of pcnt 0 and pcnt 4, so the registers update at
//  the clk edges ending pcnt 7 (7M = 0) and pcnt 3 (7M = 1), each with the
//  count's own 1H/2H/4H.  This is not a free choice: only pre-edge makes
//  /UTIME low for exactly the whole of h = 0 and h = 1, and only pre-edge puts
//  the four PIN15 terms one per half-count across h = 0 and h = 1 (term 4
//  first, then 3, 2, 1).  Post-edge sampling misses both by half a count.
//
//  ---- /RTIME --------------------------------------------------------------
//  /RTIME is high at h = 7 and low at h = 0 and h = 1: the set term fires at
//  the end of h = 7, and the hold terms carry it to the middle of h = 2.  That
//  half-count tail past h = 2 is what the hold terms literally produce, and
//  the 68000 has already latched DTACK by then.
//
//  ---- byte lanes ----------------------------------------------------------
//  1H = 0 selects the write muxes' A inputs (D15:8) and puts A0 = 0 on the
//  RAM, so the RAM even byte is the 68000's D15:8 -- big-endian, as the 68000
//  expects.  A byte write with only /UDS writes only the h = 0 byte; only /LDS
//  writes only the h = 1 byte; a word write asserts both and gets both cycles.
//
//    /RAMWP = 17J F32 OR( GAL 21R pin 15 , MODCLK )   -> 22V /WE
//  and MODCLK (= 7M delayed a quarter period, 50 % duty) is low over
//  pcnt 2..5, so /RAMWP is one clean ~70 ns pulse inside each H count.  The
//  SRAM latches on its rising edge, at the end of pcnt 5.
//
//  ---- access phasing ------------------------------------------------------
//  One access per 7M count, never a dual-port shortcut.  The read for count k
//  is issued at pcnt 6 of count k-1 and captured at pcnt 7, so `ramd` is
//  stable for every clk_sys cycle of count k, as the real address muxes settle
//  before the count too.  The write for count k happens in the /RAMWP window,
//  pcnt 2..5 of that count, so the two never share a clk_sys cycle and the
//  write always precedes the next count's read.
//
//  ---- the video port ------------------------------------------------------
//    ramd[15:8] = the 19N LS374 latch  (the even count's byte, clocked 1H^)
//    ramd[7:0]  = the live odd byte
//  so ramd is a complete word only during odd counts:
//    h mod 8 == 3 -> PF word   (ramd_pf_stb)
//    h mod 8 == 5 -> MO word bank {8H,0}  (ramd_mo0_stb)
//    h mod 8 == 7 -> MO word bank {8H,1}  (ramd_mo1_stb)
//  Each strobe is high for exactly that whole count = one ce_7m period.
//============================================================================

module badlands_vram #(
	parameter int H_TOTAL = 456          // counts of 7M per line
)(
	input  logic        clk,             // clk_sys, 57.272724 MHz
	input  logic        reset,           // synchronous, active high
	input  logic        ce_7m,           // clk_sys/8 -- the H-counter tick
	input  logic        ce_14m,          // clk_sys/4 -- part of the port
	                                     // contract; the GAL phase comes from ce_7m

	// ---------------- video side -----------------------------------------
	input  logic  [8:0] h,               // the H counter, 0..H_TOTAL-1
	// Only V[7:3] (128V..8V) reach the 26T/28T/23T address multiplexers:
	// V[2:0] select the row inside an 8-pixel stamp and V[8] does not exist
	// on the 22V at all.  The full counter stays in the port list, so the
	// unused bits are deliberate.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic  [8:0] v,
	/* verilator lint_on UNUSEDSIGNAL */
	output logic [15:0] ramd,            // RAMD15:0
	output logic        ramd_pf_stb,
	output logic        ramd_mo0_stb,
	output logic        ramd_mo1_stb,

	// ---------------- 68000 side -----------------------------------------
	input  logic        as,              // active-high copy of /AS  (21R pin 3)
	input  logic        rw,              // 1 = read                 (21R pin 4)
	input  logic        uds_n,           //                          (21R pin 5)
	input  logic        lds_n,           //                          (21R pin 6)
	input  logic        ram_n,           // /RAM, not /AS qualified  (21R pin 12)
	input  logic [12:1] cpu_a,
	input  logic [15:0] cpu_din,
	output logic [15:0] cpu_dout,

	// ---------------- to badlands_dtack ----------------------------------
	output logic        rtime_n,         // 21R pin 14 -> 19J LS08 pin 2

	// ---------------- the remaining real 21R pins, published ----
	output logic        utime_n,         // 21R pin 16
	output logic        ramre_n,         // 21R pin 17
	output logic        ramwe_n,         // 21R pin 18
	output logic        ramoe_n,         // 21R pin 19
	output logic        ramwp_n          // 17J F32 -> 22V /WE
);

	// ------------------------------------------------------------ the phase
	// pcnt == 7 is the ce_7m cycle (the count's last clk_sys cycle); the H
	// counter advances at the end of it.  `7M` is high over pcnt 0..3.
	logic [2:0] pcnt;
	always_ff @(posedge clk) begin
		if (ce_7m) pcnt <= 3'd0;
		else       pcnt <= pcnt + 3'd1;
	end

	// The two 14M rising edges of a count land at the starts of pcnt 0 and
	// pcnt 4, so the GAL registers update at the clk edges ending pcnt 7 and
	// pcnt 3, sampling the values current in that cycle (pre-edge).
	wire gal_ce   = (pcnt == 3'd3) | (pcnt == 3'd7);
	wire gal_s7m  = (pcnt == 3'd3);          // the sampled `7M` level

	// MODCLK: 23R half 1, D = 7M, CK = /14MP rising (starts of pcnt 2 and 6),
	// so MODCLK is low over pcnt 2..5 and high over pcnt 6,7,0,1.
	wire modclk = ~((pcnt >= 3'd2) & (pcnt <= 3'd5));

	// ------------------------------------------------- the GAL's input names
	wire s1h = h[0], s2h = h[1], s4h = h[2];
	wire ram = ~ram_n;
	wire uds = ~uds_n;
	wire lds = ~lds_n;
	wire wr  = ~rw;
	wire rd  =  rw;

	// ------------------------------------------------ GAL 21R, registered
	logic rtime_q, pin15_q, utime_q, ramre_q;

	wire rtime_d =
	      ( s1h & ram & rtime_q )                                            // hold a
	    | ( ~s2h & ram & rtime_q )                                           // hold b
	    | ( ~gal_s7m & as & lds & s1h & s2h & s4h & ram )                    // set, low lane
	    | ( ~gal_s7m & as & uds & s1h & s2h & s4h & ram );                   // set, high lane

	wire pin15_d = ~(
	      (  gal_s7m & wr & lds &  s1h & ~s2h & ~s4h & rtime_q )
	    | ( ~gal_s7m & wr & lds & ~s1h & ~s2h & ~s4h & rtime_q )
	    | (  gal_s7m & wr & uds & ~s1h & ~s2h & ~s4h & rtime_q )
	    | ( ~gal_s7m & as & wr & uds & s1h & s2h & s4h & ram ) );

	wire utime_d = ( gal_s7m & ~s2h & ~s4h )
	             | ( ~s1h & ~s2h & ~s4h )
	             | ( ~gal_s7m & s1h & s2h & s4h );

	wire ramre_d = as & rd & ram;

	always_ff @(posedge clk) begin
		if (reset) begin
			rtime_q <= 1'b0;
			pin15_q <= 1'b1;      // pin 15 idles high (no write qualified)
			utime_q <= 1'b0;
			ramre_q <= 1'b0;
		end else if (gal_ce) begin
			rtime_q <= rtime_d;
			pin15_q <= pin15_d;
			utime_q <= utime_d;
			ramre_q <= ramre_d;
		end
	end

	assign rtime_n = ~rtime_q;
	assign utime_n = ~utime_q;
	assign ramre_n = ~ramre_q;

	// ------------------------------------------ GAL 21R, combinatorial pins
	assign ramoe_n = ~( (as & rd & ram) | (rtime_q == 1'b0) );
	assign ramwe_n = ~( as & wr & ram & utime_q & ramoe_n );

	// 17J F32: /RAMWP = OR( 21R pin 15 , MODCLK )
	assign ramwp_n = pin15_q | modclk;

	// ------------------------------------------------- the address multiplexers
	// 26T/28T F153 select on (4H,2H); 23T F157 selects on 2H with /E = 4H;
	// 21T F157 selects on /UTIME.
	//
	// /UTIME is low for the whole of h = 0 and h = 1, i.e. exactly
	// `hh[2:1] == 2'b00`.  That equivalent form is used here so the one-count
	// look-ahead read can compute the next count's address before that count's
	// register update happens.
	/* verilator lint_off UNUSEDSIGNAL */
	function automatic logic [12:0] vram_addr
		(input logic [8:0] hh, input logic [8:0] vv, input logic [12:1] ca);
		logic a12, a11, a10, a9, a8, a7, a6, a5, a4, a3, a2, a1, a0;
		logic h4, h2, utime_lvl;
		begin
			h4 = hh[2];  h2 = hh[1];  a0 = hh[0];      // 22V A0 = 1H, always
			utime_lvl = (hh[2:1] == 2'b00);            // /UTIME asserted

			// 26T 74F153 -> A12, A6
			case ({h4, h2})
				2'b00:   begin a12 = ca[12];  a6 = ca[6];  end
				2'b01:   begin a12 = 1'b0;    a6 = hh[8];  end   // 256H
				default: begin a12 = 1'b1;    a6 = hh[8];  end
			endcase

			// 28T 74F153 -> A8, A7
			case ({h4, h2})
				2'b00:   begin a8 = ca[8];  a7 = ca[7];  end
				2'b01:   begin a8 = vv[4];  a7 = vv[3];  end     // 16V, 8V
				default: begin a8 = hh[3];  a7 = hh[1];  end     // 8H, 2H
			endcase

			// 23T 74F157 (SEL = 2H, /E = 4H) -> A11, A10, A9, A1
			if (h4)      begin a11 = 1'b0;   a10 = 1'b0;   a9 = 1'b0;   a1 = 1'b0;   end
			else if (h2) begin a11 = vv[7];  a10 = vv[6];  a9 = vv[5];  a1 = hh[3];  end
			else         begin a11 = ca[11]; a10 = ca[10]; a9 = ca[9];  a1 = ca[1];  end

			// 21T 74F157 (SEL = /UTIME, /E = GND) -> A5, A4, A3, A2
			if (utime_lvl) begin a5 = ca[5];  a4 = ca[4];  a3 = ca[3];  a2 = ca[2];  end
			else           begin a5 = hh[7];  a4 = hh[6];  a3 = hh[5];  a2 = hh[4];  end

			vram_addr = {a12, a11, a10, a9, a8, a7, a6, a5, a4, a3, a2, a1, a0};
		end
	endfunction
	/* verilator lint_on UNUSEDSIGNAL */

	localparam logic [8:0] H_LAST = H_TOTAL[8:0] - 9'd1;
	wire [8:0] h_nxt = (h == H_LAST) ? 9'd0 : (h + 9'd1);

	wire [12:0] addr_this = vram_addr(h,     v, cpu_a);
	wire [12:0] addr_next = vram_addr(h_nxt, v, cpu_a);

	// ---------------------------------------------------- the write data path
	// 28L / 22N LS257, SEL = 1H : 0 -> A inputs (D15:8), 1 -> B inputs (D7:0)
	wire [7:0] wr_byte = h[0] ? cpu_din[7:0] : cpu_din[15:8];

	// The SRAM latches on the RISING edge of /RAMWP, i.e. at the end of the
	// MODCLK window (pcnt 5).
	wire do_write = (pcnt == 3'd5) & ~ramwp_n;

	// ---------------------------------------------------------------- the SRAM
	logic [7:0] mem [0:8191];
	logic [7:0] ram_q;
	wire [12:0] ram_addr = do_write ? addr_this : addr_next;

	always_ff @(posedge clk) begin
		if (do_write) mem[ram_addr] <= wr_byte;
		ram_q <= mem[ram_addr];
	end

	// --------------------------------------------------- the read-side latches
	// byte_cur is the byte the SRAM presents for the whole of the current
	// count; the fetch that produced it was issued at pcnt 6 of the count
	// before.  During a CPU write the LS257s -- not the SRAM -- drive the RAM
	// data bus (/RAMOE is high), so the bus carries wr_byte then.
	logic [7:0] byte_cur;
	logic [7:0] latch_19n;

	wire        bus_is_write = ~ramwe_n;
	wire [7:0]  bus_byte     = bus_is_write ? wr_byte : byte_cur;

	always_ff @(posedge clk) begin
		if (reset) begin
			byte_cur  <= 8'h00;
			latch_19n <= 8'h00;
			cpu_dout  <= 16'h0000;
		end else begin
			if (pcnt == 3'd7) begin
				// 19N LS374, CK = 1H rising: fires at the boundary into an odd
				// count, capturing the byte the even count put on the bus.
				if (!h[0]) latch_19n <= bus_byte;

				// 22K LS374 is a register, CK = /4HD1H rising = the start of
				// h = 1, taking the upper byte 19N caught at the end of h = 0
				// (19N clocks on the same edge, so the value is that byte).
				if (h[2:0] == 3'd0) cpu_dout[15:8] <= bus_byte;

				byte_cur <= ram_q;
			end

			// 28N is a 74F373 transparent latch, not a register: LE = /2H, so
			// D7:0 follows the RAM bus for the whole of h = 0 and h = 1 and
			// only closes at the start of h = 2.  Modelling it as a flop
			// clocked at the end of h = 1 loses the byte outright, because the
			// 68000 samples its data at the falling edge of S6 -- the
			// h = 1 -> h = 2 boundary, the very edge the flop would use.
			if (!h[1]) cpu_dout[7:0] <= bus_byte;
		end
	end

	// -------------------------------------------------------- the video port
	assign ramd         = {latch_19n, bus_byte};
	assign ramd_pf_stb  = (h[2:0] == 3'd3);
	assign ramd_mo0_stb = (h[2:0] == 3'd5);
	assign ramd_mo1_stb = (h[2:0] == 3'd7);

	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused_ce14m = ce_14m;   // the port contract carries it; the phase
	                               // comes from ce_7m, so no second tick is used
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
