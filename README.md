# Bad Lands for MiSTer FPGA

A hardware recreation of Atari Games' 1989 arcade title **Bad Lands**,
implemented as an FPGA core for the
[MiSTer](https://github.com/MiSTer-devel/Main_MiSTer/wiki) platform.

This is not a software port. The core reconstructs the original PCB —
**A047193-01** (schematic 047194-01 rev C) — from Atari's own schematic package
in the TM-344 kit manual and from the six GAL16V8 fuse dumps that ship in the
ROM set, with MAME used as a functional oracle where the drawings are silent.
Where a schematic and MAME disagree, the schematic wins and the disagreement is
recorded — see
[Where this core differs from MAME](#where-this-core-differs-from-mame).

Bad Lands is a **single-board** design. Unlike its Atari siblings there is no
separate audio PCB: the 6502, the YM2151 and the whole analogue tail sit on the
game board next to the 68000. It shares the **SOS-2** sync generator, the
**VMATCH** motion-object comparator and the Fujitsu **MB87316** line buffer with
the Blasteroids and Xybots boards, and the **LETA** analogue-input custom with
Blasteroids and Atari System 2. There is no SLAPSTIC protection chip.

## Supported ROM sets

| MRA | MAME set | Region |
| --- | --- | --- |
| `Badlands.mra` | `badlands` | World, Atari Games |

The three bootleg sets (`badlandsb`, `badlandsb2`, `badlandsm`) are **not
supported**. They are a different PCB with a Z80 sound section and their own
GALs, and there is no MRA for them.

## Requirements

| Item | Requirement |
| --- | --- |
| Board | DE10-Nano (Cyclone V `5CSEBA6U23I7`) |
| SDRAM | **Required** — a standard 32 MB module or larger |
| Display | Horizontal cabinet, **336 × 240 visible at 59.93 Hz**, 15 kHz |
| ROMs | Supplied by you from a MAME archive; none are included here |

**The SDRAM module is not optional.** The graphics ROMs live in it — 384 KB of
playfield stamps plus 192 KB of motion-object stamps — and there is no
block-RAM fallback for them. Everything else (the 256 KB 68000 program, the
64 KB sound ROM, all RAM and the EEPROM) is on-chip. The controller is addressed
for the usual MT48LC16M16-class part (4 banks × 13-bit row × 9-bit column), so a
legacy 8 MB module will not work even though little of the module is used.

## Installation

1. Copy `releases/Badlands_YYYYMMDD.rbf` to `_Arcade/cores/` on the SD card.
2. Copy `releases/Badlands.mra` to `_Arcade/`.
3. Place the matching MAME ROM archive (`badlands.zip`) in `games/mame/`.

**No ROM data is included in this repository, and none ever will be.** The MRA
builds the correct memory image from an archive you already own. With no ROMs
present the core stays on a black screen.

## Controls

The cabinet is a two-player sit-down with a **steering wheel** and a **gas
pedal** per player, one Start/Fire button per player, and two coin chutes.

| Control | Function |
| --- | --- |
| Analog stick, D-pad, paddle, spinner, mouse | **Steering wheel** |
| R | **Gas** |
| A, B | **Fire / Start** |
| Start | Fire / Start |
| Select | Coin |

**Steering needs no setup.** Every control steers at once: the analog stick,
D-pad, paddle, spinner and (player 1 only) mouse X are added together. Many USB
spinners appear to MiSTer as a mouse; those steer player 1 through the mouse
axis. Two OSD settings adjust the feel — see [Options](#options).

**A, B and Start are the same button.** The board wires each player's Start and
Fire to a single input pin, so all three reach the same signal. That is the
real cabinet, not a simplification.

**The gas pedal is a switch, not an axis.** On the PCB the pedal gates a
free-running LM556 oscillator whose pulses clock the LETA's pedal channel, so
holding the button is exactly what pressing the pedal microswitch does.

## Options

Bad Lands has **one 4-position DIP, SW1**, and it carries no game settings.
Coinage, difficulty, bookkeeping and the high-score table all live in the
on-board **Xicor 2816A EEPROM** and are edited from the game's own self-test,
reached with the OSD **Service** toggle. Settings and high scores persist
through MiSTer's *Save Settings* (the MRA declares a 2048-byte NVRAM). On a
first boot with no save file the EEPROM reads erased, and the game writes its
own defaults — as a factory-new board does.

| OSD item | Effect |
| --- | --- |
| Aspect ratio | Original / Full Screen / two user ratios |
| Orientation | Original / Flip |
| Scale | Normal / V-Integer / HV-Integer / Narrower HV-Integer |
| Scandoubler Fx | None / HQ2x / CRT 25% / CRT 50% / CRT 75% — anything but None turns the scandoubler on |
| **Analog alignment** (page) | CRT H-Size, CRT H-Position, Analog VGA H-Shift, Analog VGA V-Shift |
| **Steering and pedals** (page) | Spinner sensitivity, Analog stick rate, Gas pedal sense |
| **Option switch SW1** (page) | SW1-1 Freeze, SW1-2 Freeze-Step, SW1-3 Old gas pedals, SW1-4 Old steering wheels |
| Service | The cabinet self-test switch |
| Reset | Core reset |

**Steering and pedals**

- **Spinner sensitivity** (0.25x – 4x, default 1x) scales the spinner, paddle
  and mouse.
- **Analog stick rate** (Normal / High / Low / Lowest) scales the analog stick
  and D-pad only. The game itself limits how fast the wheel can turn the car;
  *Normal* reaches that limit at a little over half deflection, and *Low* and
  *Lowest* are there for fine control.
- **Gas pedal sense** (Normal / Inverted) — leave on Normal. Inverted swaps the
  pedal's count direction and exists for hardware testing only.

Wheel movement is queued and fed to the board at a bounded rate. The game reads
the wheel as an 8-bit difference once a frame, so this stops a very fast spin
from wrapping around into a turn the other way.

**Analog alignment** is the same page as in the
[Toobin'](https://github.com/MiSTer-devel/Arcade-Toobin_MiSTer),
[Klax](https://github.com/MiSTer-devel/Arcade-Klax_MiSTer) and
[Xybots](https://github.com/MiSTer-devel/Arcade-Xybots_MiSTer) cores, for
centring and sizing the picture on an analog CRT. It acts on the output side
only: core timing is untouched, and at the zero settings it is a bit-exact
bypass. CRT H-Size and H-Position are 15 kHz adjustments and are bypassed while
the scandoubler runs.

**All four SW1 positions must be OFF in normal operation.** The manual lists
them as development and legacy-hardware options, not game settings — the Freeze
pair stops and single-steps the game, and the two "old" switches select earlier
control hardware. SW1-3 in particular tells the program that a different pedal
type is fitted, so the throttle misbehaves with it ON. They are exposed because
they exist on the board.

## Hardware baseline

| Block | Implementation |
| --- | --- |
| Main CPU | Motorola **68000** @ 7.159090 MHz (14.318181 MHz ÷ 2) — fx68k, cycle-exact at the microcode level |
| Protection | None — no SLAPSTIC; the 28C socket is not populated |
| Program ROM | 256 KB (four 27512s), entirely in block RAM |
| Sound section | On the game board: 6502 @ 1.789772 MHz + YM2151 @ 3.579545 MHz + YM3012, a 64 KB banked ROM and 8 KB of RAM |
| Main ↔ sound link | Two LS374 mailbox latches with a flag flop each — none of them has a reset, as on the board |
| Raster | SOS-2 (137550-001): 456 × 262 total, **336 × 240 visible**, 7.159090 MHz pixel clock, 15.70 kHz / 59.93 Hz |
| Playfield | 64 × 32 map of 8 × 8, 4 bpp stamps; six 27512s, one enabled per fetch; banked by a single flop |
| Motion objects | **32** four-byte entries; per-line vertical match in the **VMATCH** custom (137552-001); Fujitsu **MB87316** line buffer; **at most 27 objects drawn per line** |
| Priority | GAL16V8 9N (`136074-1004`), from the dumped fuse map |
| Palette | 256 entries, two bytes each, through the board's R-2R ladders: **R 5 bits, G 6 bits, B 5 bits** |
| Analogue inputs | **LETA** (137304-2002), four channels — two quadrature steering wheels and two LM556 pedal oscillators |
| NVRAM | Xicor **2816A** 2 KB parallel EEPROM, one-shot unlock strobe, lock after write |
| Watchdog | LS197 counter chain, 8 VBLANK periods (≈ 134 ms) |
| Interrupts | IRQ1 = VBLANK, IRQ2 = sound mailbox; the board cannot present any higher level |
| GALs | All six GAL16V8s — address decode, video-RAM timing, line-buffer control, priority, sound decode, horizontal strobes — implemented **verbatim** from the dumped fuse maps |

The whole core runs in a single **57.272724 MHz** `clk_sys` domain (4 × the
14.318181 MHz board crystal) with integer clock enables — ÷4, ÷8, ÷16, ÷32 — so
every clock on the board is an exact subdivision and there is no clock-domain
crossing anywhere inside the game logic.

Resource use on the DE10-Nano's Cyclone V (`5CSEBA6U23I7`): 13,617 / 41,910 ALMs
(32 %), 433 / 553 RAM blocks (78 %), 45 / 112 DSP blocks (40 %). Timing closes
on all four corners with +2.315 ns worst-case setup slack on `clk_sys`.

## Accuracy and verification

This core was developed with AI assistance. Because that raises a fair question
about whether the result was actually checked, here is the verification record.
The test suite lives in the development repository and is not shipped here;
these are its results for the released build.

**Simulation suite** — 40 gated testbenches (Icarus Verilog, plus Verilator
co-simulations that link the vendored VHDL CPU), covering the address decode,
bus timing, interrupts and watchdog, EEPROM, inputs and steering, the SDRAM
path, every video stage and the full sound section, each paired with a
warning-clean Verilator lint. Gates are required to be non-vacuous: several run
a deliberately broken mutant first and fail if the mutant passes.

**Schematic provenance** — all six schematic sheets were transcribed before any
RTL was written, and every strobe in the memory map has an address-bit equation
with a sheet and IC citation behind it. The six GALs were decoded with MAME's
`jedutil` and their equations are implemented verbatim.

**Main CPU boot** — the 68000 bus is co-simulated against MAME: **912,548
consecutive bus cycles agree** (address, R/W, data, size) through reset, the
ROM checksum, the colour-RAM fill, the EEPROM verify and the first VBLANK
interrupts. The only divergence is the pedal channel, a deliberate difference
(see below).

**Video** — MAME is instrumented to capture complete video state (playfield,
motion-object and colour RAM) at chosen frames; those states are replayed
through the real core RTL and compared to MAME's own rendered bitmap pixel for
pixel. **21 frames — 8 attract, 12 gameplay, 1 power-on — are pixel-exact over
the 1,122,902 compared pixels.** The pixels left out are the scanlines where the
CPU writes colour RAM mid-line, which the board renders differently from MAME
(see the table below).

**Sound** — the sound section is compared to MAME over **12 seconds / 719
frames**: 586 mailbox commands and 584 responses order-exact, the YM2151
register writes order- and value-exact, and 4 sound interrupts per frame. The
one divergence is the inverted self-test bit described below; flipping that
single bit reproduces MAME's streams exactly, which shows it is the *only*
difference.

**Whole-core co-simulation** — the integrated core (fx68k + T65 + jt51 + the
video pipeline + an SDRAM model), fed the real MRA download image: boot, the
sound round trip in both directions, and the same attract frames pixel-exact
out of SDRAM.

**On real hardware** (DE10-Nano): the released bitstream has been tested by many
users on their own setups, with no outstanding issues reported. The first
hardware runs during development found two faults
that every simulation gate had passed: a missing `/UDS` term on the colour-RAM
write strobe (every colour came out as one of two greens), and the pedal
channels counting in the wrong direction (full throttle at rest). Both were
traced to the schematic and the game code, fixed, and given gates of their own.

## Where this core differs from MAME

MAME is this project's functional oracle and every gate above measures the core
*against* it, so this is a narrow list, not a general claim. It is where the
A047193-01 schematics document something MAME's model does not. None of these
changes how the game plays.

| Behaviour | MAME | This core |
| --- | --- | --- |
| **Motion-object entries** | 64 entries at a 2-byte stride | The video-side address bit `A1` is forced low, so entries are **4 bytes apart and there are 32**. MAME's even entries are the hardware's entries; its odd entries do not exist on the board |
| **Objects per line** | No per-line limit | The entry number comes from the horizontal counter, so at most **27 objects are drawn on a line**, plus a 28th clipped to 6 pixels |
| **Palette bit 15** | Adds an LSB to all three guns | The ladder LSB is grounded on red and blue and wired only on green: **R 5, G 6, B 5 bits**. A small shade difference on entries with bit 15 set |
| **Colour-RAM writes repeat pixels** | No equivalent | The pixel clock is held while the CPU accesses colour RAM, so **3 consecutive pixels hold the previous colour** when that happens during active display |
| **Video / work RAM wait states** | None | The board's: the 68000 waits up to 8 pixel-clock periods (1.1 µs) per access for its time slot |
| **Sound self-test bit polarity** | D7 = 1 when self-test is *off* | The inverting LS240 and the game ROM both give D7 = 1 when the switch is *on*. MAME's polarity makes the sound CPU run its self-test during normal play and skip it in self-test |
| **Gas pedals** | A per-frame ±1 counter | The actual circuit: an LM556 oscillator whose frequency the pedal changes (≈ 280 Hz idle to ≈ 7.5 kHz pressed), clocking the LETA's pedal channel downward |
| **Sound IRQ phase** | Scanlines 0, 64, 128, 192 | The schematic's rising edges of `VBLANK OR 32V`: scanlines 32, 96, 160, 224 — the same four per frame, 32 lines later |
| **Watchdog period** | A library-default timer, about 20× too long | 8 VBLANK periods (≈ 134 ms), then reset held for 8 more frames |
| **MO stamp-row carry** | The whole 12-bit code is incremented | The carry stops at bit 9; the top two bits go to the ROM chip select untouched. They differ only if `code[9:0] + row` crosses 1024 |
| **Sound address mirrors** | `mirror(0x5f9)` / `mirror(0x5fe)` | The hardware's masks, `$01F9` and `$07FE`. No known software depends on either |

**The 27-object limit and the 32-entry list** are the two places where a crowded
screen could in principle look different from MAME. Across a 12,000-frame
two-player session the game never allocates more than 21 entries, so in practice
the two models agree; the limit is implemented because it is what the counter
does.

**The 6-bit green.** The board's green ladder has one more step than red or
blue. On a colour word with bit 15 set, MAME brightens all three guns and the
board brightens only green.

**The pedal model and the self-test bit** are cases where MAME's model is not a
model of this circuit. Neither changes what the game does.

## Known deviations and open questions

Everything below is settled as far as documents and a DE10-Nano can settle it.
What is left needs a real Bad Lands PCB. **If you have a board and are willing
to look at any of these, please open an issue.**

**1. The motion-object line-wrap window.** No load strobe fires for list entry
28, which leaves a 10-count window at the extreme left of the line in which the
line buffer could be written at a stale address. The core models it as the
schematic implies. The game parks the entry involved off-screen throughout
gameplay, so the only reachable case is the first few seconds after power-on
(10 pixels on one frame). Whether a real MB87316 behaves this way is unknown.

**2. MB87316 internals.** There is no public data sheet for the line buffer. Its
alignment is proved pixel-exact against MAME over 21 frames; its idle output
value, read latency and strobe widths as silicon facts are not.

**3. Two of the six GAL dumps have no second read.** `136074-1004` (priority,
9N) and `136074-2000` (horizontal strobes, 26R) have a single dump each; the
other four have two byte-identical dumps. Both are covered by the pixel-exact
frame comparison, so this is residual risk rather than an unknown.

**4. Steering harness wiring.** The schematic does not tie the encoder's two
phase wires to connector pins. It cannot affect this core — the steering
direction was settled from the game code — but it matters to anyone wiring a
real wheel.

**5. Two board-level oddities.** A 0 Ω `R6` is drawn, and listed in the parts
list, bridging the two coin-counter outputs, and `JAM-L` is drawn as −5 V where
the manual puts the speaker return. Neither reaches the digital core.

## Building

Quartus Prime **17.0.2** (Lite or Standard) — the version MiSTer standardises
on. Open `Badlands.qpf` and run a full compile, or from a shell:

```
quartus_sh --flow compile Badlands
```

The bitstream lands in `output_files/Badlands.rbf`. Source files are listed in
`files.qip`, which Quartus reads but cannot edit — add and remove entries by
hand. `clean.bat` removes every build product.

One line in `files.qip` must not be removed:

```
set_global_assignment -name SEARCH_PATH rtl/lib/fx68k
```

fx68k loads its microcode (`microrom.mem`, `nanorom.mem`) by relative path.
**Quartus does not error when those files are missing**: the ROMs synthesise
empty, the 68000 executes nothing, and the core comes up on a black screen.
Analysis & Synthesis prints `Info (10905)` naming each `.mem` file it read —
check for it after changing the project.

## Attribution

This core's own RTL is **GPL-3.0-or-later** (`LICENSE`). Individual third-party
components keep their own notices, which must be preserved; `NOTICE.md` is the
full list.

| Component | Author | License |
| --- | --- | --- |
| MiSTer framework (`sys/`) | Till Harbaum, Alexey Melnikov (sorgelig) and MiSTer-devel contributors | GPL-2.0-or-later / GPL-3.0-or-later per file |
| fx68k (68000) | Jorge Cwik (ijor), <https://github.com/ijor/fx68k> | GPL-3.0-or-later |
| T65 (6502) | Daniel Wallner, with fixes by Mike Johnson, Wolfgang Scherr and Morten Leikvoll (OpenCores / FPGAArcade) | BSD-style |
| jt51 (YM2151) | Jose Tejada (jotego), <https://github.com/jotego/jt51> | GPL-3.0-or-later |
| `analog_hsize` | Umberto Parisi (rmonic79), Arcade-Raiden_MiSTer | GPL-3.0-or-later |
| Altera/Intel PLL megafunction | Intel Corporation (generated IP) | Intel FPGA IP licence, as generated |

The SOS-2 raster, the LETA model, the memory path and the VMATCH / line-buffer
motion-object chain follow the author's
[Blasteroids](https://github.com/MiSTer-devel/Arcade-Blasteroids_MiSTer) core,
which is built around the same Atari customs. The fx68k wrapper and the MiSTer
glue come from the
[Xybots](https://github.com/MiSTer-devel/Arcade-Xybots_MiSTer) and
[Vindicators](https://github.com/MiSTer-devel/Arcade-Vindicators_MiSTer) cores,
the SDRAM path from
[Toobin'](https://github.com/MiSTer-devel/Arcade-Toobin_MiSTer), and the
quadrature generator and steering front end from the
[Atari System 2](https://github.com/MiSTer-devel/Arcade-AtariSystem2_MiSTer)
core.

Thanks to the MAME team, whose `badlands.cpp`, `badlands_v.cpp` and
`atarimo.cpp` served as the functional oracle throughout — and whose PCB notes
record the real-board sync frequencies that settled the 262-line raster — and
to whoever preserved the Atari TM-344 manual and schematic package and dumped
the GALs. Thanks also to everyone who tested the core on their own hardware
before release.

Bad Lands and its ROMs, artwork and manuals are © Atari Games and its
successors. This project contains no copyrighted ROM data.

Core by **RetroShrimp**.
