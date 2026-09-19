derive_pll_clocks
derive_clock_uncertainty

# ============================================================================
#  Bad Lands core timing constraints.
#
#  There is exactly ONE fabric clock in the game half of this design:
#
#     clk_sys = 57.272724 MHz  (emu|pll outclk_0 = PLL 50 MHz x 63 / 5 / 11)
#             = 4 x 14.318181 MHz, the A047193-01 master crystal.
#               Period 17.461 ns.
#
#  Everything below it advances on synchronous, integer clock ENABLES generated
#  in rtl/badlands_core.sv from one free-running /32 counter:
#
#     ce_14m = clk_sys / 4   = 14.318181 MHz  (CRAM byte rate, GAL 21R clock)
#     ce_7m  = clk_sys / 8   =  7.159091 MHz  (pixel, SOS-2 H counter, 68000)
#     ce_1h  = clk_sys / 16  =  3.579545 MHz  (YM2151 phiM, LETA CK)
#     ce_2h  = clk_sys / 32  =  1.789772 MHz  (6502 phi0)
#
#  Each slower enable is a strict subset of every faster one, so no fractional
#  or asynchronous enable exists anywhere and there is no CDC inside the game
#  logic.  outclk_1 is the same 57.272724 MHz, phase-shifted, for SDRAM_CLK.
#
#  ---- the rule every multicycle exception below obeys ----------------------
#  Without them the analyser holds the big CE-gated vendored cores to the full
#  17.461 ns period, which is FALSE for registers that provably share one clock
#  enable.  Only INTRA-block reg->reg paths are relaxed, and only inside a
#  hierarchy where EVERY sequential process takes the same Enable.  A CE-gated
#  block's OUTPUTS to full-rate logic stay single-cycle: a full-rate sampler may
#  legitimately latch a held output on any clk_sys edge.  Nothing that runs at
#  the full rate — the SDRAM controller, the graphics arbiter, the motion-object
#  engine, the video pipeline, the 22V slot logic, the bus decode — gets an
#  exception of any kind.
#
#  Only three hierarchies qualify: fx68k (four narrow author-supplied exceptions
#  and nothing else), T65 and jt51.  Each block below is guarded by a presence
#  test on its own registers and says in the STA log which way it went, so a
#  constraint that stops matching because a register was renamed shows up as
#  "not in this build" instead of disappearing in silence.
# ============================================================================

# ============================================================================
#  MAIN CPU — fx68k 22E (rtl/lib/fx68k, instance u_core|u_main|u_cpu|u_fx68k)
# ============================================================================
# These are Jorge Cwik's OWN constraints, quoted from
# core/rtl/lib/fx68k/fx68k.txt "Timing analysis" ("Microcode access is one of
# the slowest paths on the core.  But the microcode output is not needed
# immediately"), with only the hierarchy completed for this project's instance
# — which that file says is the part that must be modified.  They are `-start`
# in the original; for a single clock `-start` and `-end` are the same
# relationship, so the form is left as the author wrote it.
#
# Why they are true here, checked in the source rather than taken on trust:
#   * `Ir` is written only under `enT1`            (fx68k.sv:318-322)
#   * `microAddr` and `nanoAddr` only under `enT1` (fx68k.sv:260-266)
#     — their other leg is the `Clks.pwrUp` cold-reset load, which is not in
#       this `-from` collection.
#   * `enT1 = enPhi1 & (tState == T4) & ~wClk`     (fx68k.sv:178)
#     and one pass of the T-state machine alternates enPhi1/enPhi2, which
#     rtl/main/badlands_cpu.sv places EXACTLY 4 clk_sys apart inside a clk_sys/8
#     CPU clock.  So successive `enT1` pulses are 16 clk_sys apart — one 68000
#     micro-cycle = two clock periods — and longer whenever the 28R sequencer
#     inserts waits.
# The launch and the capture are the same enable, 16 clk_sys apart, and the
# exception claims 2.  The combinational cloud it covers is
# rtl/lib/fx68k/uaddrPla.sv, the opcode PLA.
set fx68k_ir [get_registers -nowarn {*u_main|u_cpu|u_fx68k|Ir[*]}]
if {[get_collection_size $fx68k_ir] > 0} {
	post_message -type info "Badlands SDC: fx68k main CPU present -- applying fx68k.txt microcode multicycle exceptions"
	set_multicycle_path -start -setup -from [get_registers {*u_main|u_cpu|u_fx68k|Ir[*]}] -to [get_registers {*u_main|u_cpu|u_fx68k|microAddr[*]}] 2
	set_multicycle_path -start -hold  -from [get_registers {*u_main|u_cpu|u_fx68k|Ir[*]}] -to [get_registers {*u_main|u_cpu|u_fx68k|microAddr[*]}] 1
	set_multicycle_path -start -setup -from [get_registers {*u_main|u_cpu|u_fx68k|Ir[*]}] -to [get_registers {*u_main|u_cpu|u_fx68k|nanoAddr[*]}] 2
	set_multicycle_path -start -hold  -from [get_registers {*u_main|u_cpu|u_fx68k|Ir[*]}] -to [get_registers {*u_main|u_cpu|u_fx68k|nanoAddr[*]}] 1
} else {
	post_message -type info "Badlands SDC: fx68k main CPU not in this build -- its multicycle exceptions skipped"
}

# ---- fx68k gets NO intra-block "it is all ce_7m anyway" exception -----------
# The T65 block below relaxes a whole vendored core because every retained
# update in it is qualified by one enable.  fx68k does NOT qualify: its uRom and
# nanoRom output registers (fx68k.sv:2471-2490) take NO clock enable at all and
# are re-clocked on every one of the 57.272724 MHz edges, which is what makes
# the two ROMs infer as block RAM.  A blanket -from/-to over `*u_fx68k|*` would
# therefore relax two genuinely full-rate M10K read paths, so none is applied
# and every fx68k path other than the four above is analysed at the full
# 17.461 ns period.  Those four stay safe under the same reading: they name `Ir`
# only, and `Ir` IS enable-gated.

# ============================================================================
#  SOUND 6502 — T65 (rtl/lib/T65, instance u_core|u_sound|u_bus|u_cpu|u_t65)
# ============================================================================
# Every sequential process in T65.vhd is gated by the same `Enable` input, which
# rtl/sound/badlands_snd_bus.sv drives from ce_2h = clk_sys/32 — the 6502A's own
# phi0 on this board, taken from the H counter.
# Eight cycles is deliberately conservative against the real 32-clock interval,
# and it keeps the exception away from anything that might be re-timed into it.
# The collection is the T65 core inside the wrapper only — the wrapper's own
# flat-port glue, the decode, the mailboxes, the YM bridge and the analogue
# filter are ordinary full-rate paths.
set t65_regs [get_registers -nowarn {*u_sound|u_bus|u_cpu|u_t65|*}]
if {[get_collection_size $t65_regs] > 0} {
	post_message -type info "Badlands SDC: T65 sound 6502 present -- applying the ce_2h intra-core multicycle exception"
	set_multicycle_path -setup -end 8 -from [get_registers {*u_sound|u_bus|u_cpu|u_t65|*}] -to [get_registers {*u_sound|u_bus|u_cpu|u_t65|*}]
	set_multicycle_path -hold  -end 7 -from [get_registers {*u_sound|u_bus|u_cpu|u_t65|*}] -to [get_registers {*u_sound|u_bus|u_cpu|u_t65|*}]
} else {
	post_message -type info "Badlands SDC: T65 sound 6502 not in this build -- its multicycle exception skipped"
}

# ============================================================================
#  YM2151 9D — jt51 (instance u_core|u_sound|u_ym|u_jt51)
# ============================================================================
# jt51 drives the LFO PM value, the channel frequency/modulation registers and
# the operator register file from `cen_p1`, which rtl/sound/badlands_ym.sv wires
# to ce_2h; the phase generator's `keycode_II` destination is clocked by the
# same enable.  At 57.272724 MHz successive cen_p1 pulses are 32 fabric clocks
# apart, so these data cannot be launched and captured on adjacent clk_sys
# edges.  The exception is kept on the exact source blocks and the exact
# destination rather than relaxing all of jt51: its timer flags and the ØM-rate
# (`cen` = ce_1h) pipeline do NOT uniformly share this enable.  Two clocks is
# deliberately conservative, with the standard N-1 hold adjustment.
set jt51_kc [get_registers -nowarn {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]
if {[get_collection_size $jt51_kc] > 0} {
	post_message -type info "Badlands SDC: jt51 YM2151 present -- applying the cen_p1 keycode multicycle exceptions"

	set_multicycle_path -setup -end 2 \
	  -from [get_registers {*u_sound|u_ym|u_jt51|u_lfo|pm[*]}] \
	  -to   [get_registers {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]
	set_multicycle_path -hold -end 1 \
	  -from [get_registers {*u_sound|u_ym|u_jt51|u_lfo|pm[*]}] \
	  -to   [get_registers {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]

	set_multicycle_path -setup -end 2 \
	  -from [get_registers {*u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_ch|kc[*]* *u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_ch|kf[*]* *u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_ch|pms[*]*}] \
	  -to   [get_registers {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]
	set_multicycle_path -hold -end 1 \
	  -from [get_registers {*u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_ch|kc[*]* *u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_ch|kf[*]* *u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_ch|pms[*]*}] \
	  -to   [get_registers {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]

	set_multicycle_path -setup -end 2 \
	  -from [get_registers {*u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_op|u_reg1op|*}] \
	  -to   [get_registers {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]
	set_multicycle_path -hold -end 1 \
	  -from [get_registers {*u_sound|u_ym|u_jt51|u_mmr|u_reg|u_csr_op|u_reg1op|*}] \
	  -to   [get_registers {*u_sound|u_ym|u_jt51|u_pg|keycode_II[*]}]
} else {
	post_message -type info "Badlands SDC: jt51 YM2151 not in this build -- its multicycle exceptions skipped"
}

# ============================================================================
#  SDRAM external I/O timing (MT48LC16M16 class, CL2, 57.272724 MHz controller).
#
#  SDRAM_CLK is PLL outclk_1 (the SECOND output counter, general[1]) wired
#  straight to the pin in Arcade-Badlands.sv — a clean phase-controlled clock,
#  so these delays can actually close.  Data-sheet values, the same chip class
#  as the sibling cores.
#
#  DO NOT TUNE THE READ-CAPTURE PHASE FROM THIS REPORT.  The Setup Summary is
#  keyed by the CAPTURE clock, so the SDRAM_CLK row is the command/write LAUNCH
#  and the DQ return is buried in the clk_sys row with ordinary core logic —
#  neither isolates the capture, and the spread across refits is the same size
#  as the effect.  The phase is set by sweeping it on a board; rtl/pll/pll_0002.v
#  carries the value inherited from the tuned sibling cores.
# ============================================================================
create_generated_clock -name SDRAM_CLK \
  -source [get_pins -compatibility_mode {*|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}] \
  [get_ports {SDRAM_CLK}]

# Read capture: data access time tAC = 6.0 ns (max), output hold tOH = 2.5 ns (min).
set_input_delay  -clock SDRAM_CLK -max 6.0 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock SDRAM_CLK -min 2.5 [get_ports {SDRAM_DQ[*]}]

# Command/address/data launch: input setup tIS = 1.5 ns (max), hold tIH = 0.8 ns (min).
set_output_delay -clock SDRAM_CLK -max  1.5 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_CKE}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_CKE}]

# The controller launches and captures on clk_sys (outclk_0); SDRAM_CLK is
# phase-shifted from it, so allow the read the proper (next) capture edge
# instead of the half-cycle one.  Endpoints must be CLOCK collections
# (get_clocks), not pins.  get_clocks does Tcl string matching where [..] is a
# character class, so the literal "general[0]" is matched with "?" wildcards;
# the *|pll|pll_inst| prefix disambiguates it from the framework's audio PLL.
set_multicycle_path -setup -end 2 \
  -from [get_clocks {SDRAM_CLK}] \
  -to   [get_clocks {*|pll|pll_inst|altera_pll_i|general?0?.gpll~PLL_OUTPUT_COUNTER|divclk}]
set_multicycle_path -hold -end 1 \
  -from [get_clocks {SDRAM_CLK}] \
  -to   [get_clocks {*|pll|pll_inst|altera_pll_i|general?0?.gpll~PLL_OUTPUT_COUNTER|divclk}]
