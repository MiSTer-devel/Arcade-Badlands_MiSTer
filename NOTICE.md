# NOTICE — third-party components and attributions

This repository's own RTL is **GPL-3.0-or-later** (see `LICENSE`). This file
names every third-party component compiled into the core, its authors and its
license. File headers are preserved in place.

## Compiled into the bitstream

| Component | Author(s) | License | Location |
|---|---|---|---|
| MiSTer framework (`sys_top`, `hps_io`, `arcade_video`, `video_freak`, `screen_rotate`, `ascal`, PLL config, …) | Till Harbaum, Alexey Melnikov (sorgelig), MiSTer-devel contributors | GPL-2.0-or-later / GPL-3.0-or-later per file | `sys/` |
| Altera/Intel PLL megafunction instance | Intel Corporation (generated IP) | Intel FPGA IP licence, as generated | `rtl/pll*/` |
| `crt_adjust.sv` — CRT Adjust line buffer (H-Size / H-Position / V-Shift) | Umberto Parisi (rmonic79), MiSTer-CRT-Adjust <https://github.com/rmonic79/MiSTer-CRT-Adjust> `rtl/crt_adjust.sv` at c682de9, with four adaptations listed in the file header | GPL-3.0-or-later | `rtl/video/crt_adjust.sv` |
| fx68k — cycle-exact 68000 (the main CPU), vendored verbatim with its own `LICENSE` and `README.md` | Jorge Cwik (ijor), <https://github.com/ijor/fx68k> | GPL-3.0-or-later | `rtl/lib/fx68k/` |
| T65 6502 soft CPU and the `T65_wrap` flat-port wrapper (the sound CPU) | Daniel Wallner, Mike Johnson, Wolfgang Scherr, Morten Leikvoll | BSD-style (OpenCores); the "redistributions in synthesized form" clause is satisfied by this notice | `rtl/lib/T65/` |
| jt51 YM2151 (OPM) FM core | Jose Tejada Gómez (jotego), <https://github.com/jotego/jt51> | GPL-3.0-or-later | `rtl/lib/jt51/` |

Nothing in `rtl/lib/` is edited: every vendored file is byte-for-byte
upstream, and all Bad Lands adaptation lives in the wrappers
(`rtl/main/badlands_cpu.sv`, `rtl/sound/badlands_snd_bus.sv`,
`rtl/sound/badlands_ym.sv`).

`rtl/lib/fx68k/` carries two `$readmemb` initialisers, `microrom.mem` and
`nanorom.mem`, loaded by **relative path**; `files.qip` therefore adds
`rtl/lib/fx68k` to Quartus' `SEARCH_PATH`. Neither file is modified.

## Derived work

Every file below is Bad Lands' own transcription of the A047193-01 schematics;
the right-hand column names the earlier work whose **structure** it follows, as
each file's header records. Those cores are the same author's earlier MiSTer
projects. All are GPL-3.0 except the Atari System 2 core, which is
GPL-2.0-or-later — compatible with this project's GPL-3.0.

| Bad Lands file(s) | Follows |
|---|---|
| `Arcade-Badlands.sv`, `rtl/badlands_core.sv`, `Badlands.sdc` | Vindicators / Xybots / Toobin' MiSTer cores |
| `rtl/video/badlands_sos2_sync.sv`, `badlands_testpat.sv` | Blasteroids / Xybots / Vindicators MiSTer cores |
| `rtl/video/badlands_analog_adjust.sv` | Blasteroids MiSTer core, `blstroid_analog_adjust.sv` (the crt_adjust glue), written from MiSTer-CRT-Adjust's `examples/core_side_snippet.v` (Umberto Parisi) |
| `rtl/main/badlands_main.sv`, `badlands_cpu.sv`, `badlands_addr_decode.sv`, `badlands_dtack.sv`, `badlands_irq.sv`, `badlands_watchdog.sv`, `badlands_eeprom_2816.sv`, `badlands_nvram_io.sv` | Blasteroids `blstroid_main.sv` and the Xybots fx68k CPU wrapper |
| `rtl/main/badlands_inputs.sv` — the `badlands_leta` model | Blasteroids `blstroid_leta.sv`, which itself follows the Atari System 2 core's `sys2_leta.sv` |
| `rtl/main/badlands_inputs.sv` — the `badlands_quad_gen` model | Atari System 2 MiSTer core, `sys2_quad_gen.sv` (GPL-2.0-or-later) |
| `rtl/badlands_steer.sv` | Atari System 2 MiSTer core, `sys2_analog_spin.sv` and `sys2_analog_axis.sv` (GPL-2.0-or-later); spinner sensitivity table from the Blasteroids core |
| `rtl/mem/badlands_sdram.sv`, `badlands_sdram_loader.sv`, `badlands_rom_loader.sv`, `badlands_prog_rom.sv`, `badlands_gfx_mem.sv`, `badlands_vram.sv` | Blasteroids / Xybots memory path |
| `rtl/video/badlands_pf.sv`, `badlands_vmatch.sv`, `badlands_mo_fetch.sv`, `badlands_lb.sv`, `badlands_lb_ctrl.sv`, `badlands_mo.sv`, `badlands_prio.sv`, `badlands_cram.sv`, `badlands_dac.sv`, `badlands_video.sv`, `badlands_hstrobes.sv`, `badlands_gfx_client.sv` | Blasteroids and Xybots video pipelines (the same Atari customs — VMATCH and the MB87316 line buffer) |
| `rtl/sound/badlands_sound.sv`, `badlands_snd_bus.sv`, `badlands_snd_decode.sv`, `badlands_snd_io.sv`, `badlands_mailbox.sv`, `badlands_ym.sv`, `badlands_snd_filter.sv` | The Blasteroids / Xybots / Vindicators JSA sound-board modules — structure only: Bad Lands has no JSA board, no POKEY, no TMS5220 and no stereo |

## Reference material (read for behaviour, never compiled or redistributed)

MAME's `badlands.cpp`, `badlands_v.cpp` and `atarimo.cpp` (BSD-3-Clause,
copyright Aaron Giles and the MAME contributors) were used as a functional
oracle. The Atari TM-344 manual and schematic scan, the GAL fuse dumps and all
ROM images are inputs that are **never committed or redistributed** by this
project.

## Trademarks and content

This project distributes no ROM data and no copyrighted artwork. *Bad Lands*
is a trademark of its rights holders. Use only with software you are legally
entitled to.
