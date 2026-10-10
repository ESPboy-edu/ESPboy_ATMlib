# ATMlib

**ATMlib** stands for **Arduboy Tracker Music**. It is a four-channel chiptune playroutine and song format for the [Arduboy](https://arduboy.com), based on [_**Squawk**_](https://github.com/stg/Squawk "Squawk Github Page"), a minimalistic 8-bit software synthesizer and playroutine library for Arduino created by Davey Taylor aka STG.

While _Squawk_ provides a very nice synth, it wasn't optimized for a small footprint. Songs were not very efficient in size, so Joeri Gantois aka JO3RI asked Davey to help him work on a new song format, and so ATMlib was born.

* **4 channels.** Default mix: pulse, square, saw, noise. Any channel can be compiled to pulse, square, saw, noise or off.
* **Tiny songs.** A song is a compact command stream of tracks that call each other. A good in-game loop fits in 50 to 100 bytes, a three-minute song in about 1.5 KB.
* **Song + SFX.** The background score keeps running while a one-track sound effect takes one voice and gives it back when it ends.
* **Cues.** `ATM_CUE(id)` in a score; the sketch reads it with `ATM.check()` to sync animation or gameplay.
* **Compile-time config.** Waveforms, effects, functions and octaves can be switched off to save flash and ISR cycles.
* **One document.** This README is both the user manual and the songwriting guide, for people and for AI assistants: give an assistant this file and the `ATMlib` folder and it can write songs.

**Contributors:** Davey Taylor (ATMsynth, effects), Joeri Gantois (effects, song format, TEAM a.r.g. tools).

<!-- toc:start -->
## Contents

**Using the library**

* [Quick start](#quick-start)
* [Library overview](#library-overview)
* [API](#api)
* [Sound effects](#sound-effects)
* [Cues](#cues)
* [Compile-time config](#compile-time-config)
* [Tracker editor](#tracker-editor)

**Song format and songwriting guide** (sections 0 to 13, referred to as "section N" in the text)

* [0. Start here](#0-start-here): house rules, starter prompt, workflow
* [1. Mental model](#1-mental-model)
* [2. Song data format](#2-song-data-format)
* [3. Command reference](#3-command-reference)
* [4. How the playroutine really behaves](#4-how-the-playroutine-really-behaves)
* [5. Timing, tempo and pitch](#5-timing-tempo-and-pitch)
* [6. Hop-free loops: the self-loop](#6-hop-free-loops-the-self-loop-the-most-important-section)
* [7. Fitting a byte budget](#7-fitting-a-byte-budget) (7.1 making a finished song smaller)
* [8. Composition recipes that worked](#8-composition-recipes-that-worked)
* [9. Tooling](#9-tooling)
* [10. Output file template](#10-output-file-template)
* [11. Using a song in a sketch](#11-using-a-song-in-a-sketch)
* [12. Mistakes made on this project](#12-mistakes-made-on-this-project-do-not-repeat)
* [13. Catalog of the existing songs](#13-catalog-of-the-existing-songs)

[Credits and license](#credits-and-license)
<!-- toc:end -->

---

## Quick start

1. Install ATMlib (Arduino Library Manager, or copy this folder into your Arduino `libraries` folder).
2. Copy `docs/ATMconfig.h` into your sketch folder (optional, see *Compile-time config*). Without it every effect, function and octave is compiled in.
3. Put your song in `song.h` (see the examples in `examples/`) and play it:

```cpp
#include <Arduino.h>
#include <Arduboy2.h>
#include "ATMconfig.h"      // optional
#include <ATMlib.h>
#include "song.h"           // your song: const uint8_t PROGMEM array, includes <ATMcmds.h>

Arduboy2Base arduboy;
ATMsynth ATM;

void setup() {
  arduboy.begin();
  arduboy.setFrameRate(60);
  arduboy.audio.on();       // make sure sound was not muted by a previous sketch
  ATM.play(starLight);      // the array name from song.h
}

void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();
  if (arduboy.justPressed(A_BUTTON)) ATM.playPause();
}
```

A song is an array of bytes written with the `ATM_*` macros from `ATMcmds.h`. A complete 73-byte example is in section 8.7. The format is described in sections 2 and 3, and the songwriting guide starts at section 0.

## Library overview

| File / folder | Role |
|---|---|
| `src/ATMlib.h`, `src/ATMlibimpl.h` | Playroutine, ISR mixer, the `ATMsynth` class |
| `src/ATMcmds.h` | Notes C2 to D7 and all song and effect macros (`ATM_NOTE_*`, `ATM_VOL`, `ATM_GOTO`, ...), with the bytes each one uses |
| `docs/ATMconfig.h` | Template for the compile-time options (copy it into the sketch) |
| `docs/index.html` | The documentation web page, generated from this README by `extras/tools/readme_to_html.py` |
| `examples/loops/<name>/` | Songs that loop forever, one Arduino sketch per song (`<name>.ino`, `song.h`, `ATMconfig.h`, `bitmaps.h`). `inGameSongs` holds 30 small loops in one sketch: A switches song, B pauses |
| `examples/songs/<name>/` | One-shot and long songs: Cutlass Cove, Midnight Run, Orbital Rush |
| `examples/soundEffects/` | Sound-effect examples |
| `extras/` | Reference tables (`frequencyToTone.md`, `notesInChords.md`) and `tools/`: simulator, generators, converter |

## API

Create one `ATMsynth` and call these from the sketch. Every method except `play()` can be compiled out with an `ATM_FUNC_*` flag in `ATMconfig.h`; calling a removed method is a compile error.

| Method | What it does |
|---|---|
| `play(const byte *song)` | Start a packed song from PROGMEM (header + tracks). |
| `playPause()` | Toggle pause / resume. |
| `pause()` / `resume()` | Explicit pause control. |
| `stop()` | Halt playback. |
| `muteChannel(ch)` / `unMuteChannel(ch)` | Silence or restore one of CH0 to CH3. |
| `playSfx(const byte *track, byte ch)` | Play a raw command stream on one channel. Music on the other channels continues. |
| `check()` | Last `ATM_CUE` byte, then clears it (0 if none since the last read). |
| `check(uint8_t id)` | 1 if the last cue equals `id` (does not clear). |
| `setTempo(byte t)` | Set the tempo (ticks per second, see section 5.1) for songs and sound effects. |

**Channels:** `CH_ZERO` (0), `CH_ONE` (1), `CH_TWO` (2), `CH_THREE` (3). Defaults at compile time: CH0 pulse, CH1 square, CH2 saw, CH3 noise (change them in `ATMconfig.h` and rebuild).

## Sound effects

There are two ways to play a sound effect over a running song. The sketch `examples/soundEffects/sfxDemo` has 45 effects in each style, in the same order, so you can compare them with the A and B buttons (A = poke, B = `playSfx`, LEFT / RIGHT = next effect, UP = next song, DOWN = pause). The screen shows the name and byte size of each effect.

| | `ATM.playSfx()` | Poke |
|---|---|---|
| Effect is | an ordinary ATMlib command stream (`VOL`, `SL_VOL`, `SL_FRQ`, `GLIS`, `ARP`, `VIB`, `TREM`, notes) | a list of 4-byte segments (volume fade, start frequency, slide, length) |
| Size | 7 to 19 bytes for the demo effects | 5 to 17 bytes for the demo effects |
| Timing | song ticks (follows the song tempo) | frames, 60 per second, independent of the song |
| Needs | `ATM_FUNC_SFX 1` | `ATM_FUNC_MUTE 1`, `ATM_FUNC_UNMUTE 1`, a call to `sfxUpdate()` once per frame |
| While the song is paused | works | needs a small workaround (below) |
| Sounds like | whatever the playroutine can do (arpeggios, vibrato, glissando) | simple beeps, sweeps and fades |

### `playSfx`

A one-track effect is a PROGMEM byte list that ends with `ATM_STOP_CHAN`. `ATMcmds.h` provides a wrapper:

```cpp
ATM_SFX_TRACK(sfxJump,
  ATM_VOL(50),
  ATM_SL_VOL((uint8_t)-3),   // fade out
  ATM_GLIS(0),               // glide up one semitone per tick
  ATM_NOTE_C4,
  ATM_DELAY(14)
);
// expands to: const uint8_t sfxJump[] PROGMEM = { ..., ATM_STOP_CHAN };
```

Play it with `ATM.playSfx(sfxJump, CH_ONE);`. The effect takes the channel for its duration, silences the song on that channel (even while the song is paused) and gives the channel back when it ends. Nothing else is needed: no mute calls. The usual pattern is to put hits on the noise channel (`CH_THREE`) or on a voice that is quiet in the song. `playSfx()` also works with no song playing.

* **Timing follows the song tempo.** Delays are in ticks, so the same effect lasts about 30 % shorter in a song at 52 than in a song at 38. Do not put `ATM_SET_TEMPO` in an effect: it changes the tempo of the song as well.
* **One track, no calls.** `GOTO` and `REPEAT` need other tracks, so an effect is a flat list of commands.
* **One slide slot per channel** (section 4.5): `SL_VOL` and `SL_FRQ` replace each other. For a pitch sweep with a fade use `GLIS` plus `SL_VOL`.

### Poke

The effect writes the oscillator registers directly, once per frame, instead of going through a track. `examples/soundEffects/sfxDemo/sfx.h` is the player (about 70 lines; copy it with the sketch). An effect is a list of segments and an end marker:

```cpp
SFX_SEG(m, SFX_HZ(1320), SFX_SL(0), 4)    // m = volume shift 0..3, start Hz, slide Hz/frame, frames
SFX_SEG(m, SFX_HZ(1760), SFX_SL(0), 14)
SFX_END
```

Volume at each frame is `min(63, framesLeft << m)`, so every segment fades out; `m` sets how long it stays loud (3 = short and loud, 0 = long and soft). A segment is 4 bytes, plus 1 for the end marker. Play it with `sfxPlay(effect, channel)` and call `sfxUpdate()` once per frame (it must be 60 times a second, e.g. right after `nextFrame()`).

* **Units.** The oscillator frequency unit is 31250 / 65536 = 0.4768 Hz; `SFX_HZ()` and `SFX_SL()` convert. Keep the pitch between 40 Hz and 7800 Hz and the slide within about +-240 Hz per frame (`SFX_SL` is applied four times per frame).
* **Mute.** The player mutes its channel with `ATM.muteChannel()` so the playroutine does not overwrite the values. The playroutine clears every mute when a song restarts a loop, so the mute is **re-asserted every frame** (that is why `sfxUpdate()` must run every frame), and each mute change is wrapped in `cli()` / `SREG` because the playroutine runs in an interrupt.
* **Noise channel (CH3).** Never write `osc[3].freq`: it holds the noise shift register. The player only sets the volume there (halved).
* **A running song is required.** The poke only works while the playroutine interrupt is running, so not after a one-shot song has ended; use `playSfx` there.
* **While paused.** `ATM.pause()` makes the playroutine set the volume of every channel without a `playSfx` to 0 about 38 times a second, which would chop a poked effect. The demo starts a 4-byte silent `playSfx` on the effect's channel first (`ATM.playSfx(pokeHold, ch)` with `ATM_SFX_TRACK(pokeHold, ATM_VOL(0), ATM_DELAY(40))`), which keeps the playroutine off that channel.
* Starting a poke effect while a `playSfx` runs on the same channel (or the other way round) makes one of them silent. The demo stops the poke effect (`sfxStop()`) before `playSfx`.

### Keep the music quieter than the effects

**In-game music should play at about 60 % of the volume used for title music** whenever the game uses sound effects; effects at full volume are otherwise lost behind the song. Title or menu music (no effects) can stay at 100 %. Do not scale the volume with a runtime control; scale the numbers in the song at compile time, which costs no bytes:

```cpp
#define MUSIC_VOL_PCT 60                       // 100 = original
#define MV(v) ((v) * MUSIC_VOL_PCT / 100)
// the fade step, scaled the same way and never below 1 (a 0 would stop the fade)
#define MS(n) ((uint8_t)-((((n) * MUSIC_VOL_PCT + 50) / 100) < 1 ? 1 : (((n) * MUSIC_VOL_PCT + 50) / 100)))

ATM_VOL(MV(34)), ATM_SL_VOL(MS(2)), ...        // instead of ATM_VOL(34), ATM_SL_VOL((uint8_t)-2)
```

Scale **both**: if only `ATM_VOL` is scaled, every note starts lower but fades at the old rate and dies sooner, so the song sounds clipped and drier. Scaling the fade step too keeps the shape. A fade step of 1 cannot be made smaller, so very slow fades (`SL_VOL(-1)`) still end somewhat earlier at 60 % than in the original. All five songs in `sfxDemo/` use these macros (`MUSIC_VOL_PCT` is at the top of `song.h`); `extras/tools/gen_demo_songs.py` emits them.

## Cues

Put `ATM_CUE(id)` anywhere in a score (2 bytes) and poll it from the sketch:

```cpp
uint8_t id = ATM.check();      // returns the last cue and clears it
if (ATM.check(3)) {
  // last cue was 3: e.g. fire a sprite frame (check(id) does not clear)
}
```

A cue is overwritten by the next one if the sketch does not poll often enough. Escaper Droid puts `ATM_CUE(1)` in its kick track so a droid sprite dances on the beat.

## Compile-time config

Waveforms, effects, functions and octaves are fixed at compile time. Copy `docs/ATMconfig.h` into the sketch folder as `ATMconfig.h`, edit it and rebuild. Every flag defaults to **on**, including when the sketch has no `ATMconfig.h`. `1` or `TRUE` compiles a feature in, `0` or `FALSE` leaves it out.

* **Waveforms:** `ATM_WAVE_CH0` to `ATM_WAVE_CH3` = `ATM_WAVE_PULSE` (0), `ATM_WAVE_SQUARE` (1), `ATM_WAVE_NOISE` (2, use on CH3), `ATM_WAVE_SAW` (3) or `ATM_WAVE_OFF` (255, no mixer for that channel = smaller ISR and fewer cycles). If no channel is noise, the LFSR is left out of the ISR.
* **`ATM_ALT_WIRING`:** 0 = stock Arduboy speaker on OCR4A only; 1 = also drive OCR4D (homemade or dual-pin wiring).
* **Effects, functions, octaves:** switches such as `ATM_FX_ARP`, `ATM_FUNC_SFX`, `ATM_OCTAVE_6`. `ATM.play()` is always compiled. `ATM_FUNC_PLAYPAUSE` needs pause and resume.

> **Warning:** the song bytes are never rewritten. If a flag is 0 and a song or sound effect still uses that effect, function or octave, playback breaks: the player ignores the command and does not consume its parameter bytes, a removed function will not compile, and an excluded note is silent. That is the sketch author's responsibility.

The full template, `docs/ATMconfig.h`:

```cpp
#ifndef _ATMCONFIG_H_
#define _ATMCONFIG_H_

// Waveforms are fixed at compile time. Change these, then recompile.
// ATM_WAVE_PULSE  0
// ATM_WAVE_SQUARE 1
// ATM_WAVE_NOISE  2   (use on channel 3; ATM_NOISE() is a song macro)
// ATM_WAVE_SAW    3
// ATM_WAVE_OFF    255  (no mixer for this channel — smaller ISR)

#ifndef ATM_WAVE_PULSE
#define ATM_WAVE_PULSE  0
#define ATM_WAVE_SQUARE 1
#define ATM_WAVE_NOISE  2
#define ATM_WAVE_SAW    3
#define ATM_WAVE_OFF    255
#endif

#ifndef ATM_WAVE_CH0
#define ATM_WAVE_CH0 ATM_WAVE_PULSE
#endif
#ifndef ATM_WAVE_CH1
#define ATM_WAVE_CH1 ATM_WAVE_SQUARE
#endif
#ifndef ATM_WAVE_CH2
#define ATM_WAVE_CH2 ATM_WAVE_SAW
#endif
#ifndef ATM_WAVE_CH3
#define ATM_WAVE_CH3 ATM_WAVE_NOISE
#endif

// Example: drop ch2 from the mixer
// #define ATM_WAVE_CH2 ATM_WAVE_OFF

// 0 = stock Arduboy speaker on OCR4A only
// 1 = also drive OCR4D (homemade / dual-pin wiring)
#ifndef ATM_ALT_WIRING
#define ATM_ALT_WIRING 0
#endif

// ---------------------------------------------------------------------------
// Effects, functions, octaves. 1 or TRUE = compile in. 0 or FALSE = leave out.
//
// WARNING: the tracker still writes the same song bytes. Nothing is stripped
// from a score. If a flag is 0 and a song or SFX still uses that effect,
// function, or octave, the sketch breaks: the player ignores the command and
// does not consume its parameter bytes, a removed function will not compile,
// and an excluded note is silent. That is the sketch author's responsibility.
//
// A sketch with no ATMconfig.h compiles all of these in. The 1s below match that.
// ATM.play() is always compiled. ATM_FUNC_PLAYPAUSE needs PAUSE and RESUME.
// ---------------------------------------------------------------------------

// Effects (command + per-tick code, and the channel fields they need)
#ifndef ATM_FX_VOLUME
#define ATM_FX_VOLUME 1       // ATM_VOL
#endif
#ifndef ATM_FX_VOL_SLIDE
#define ATM_FX_VOL_SLIDE 1    // ATM_SL_VOL / ATM_SL_VOL_ADV / ATM_SL_VOL_OFF
#endif
#ifndef ATM_FX_FREQ_SLIDE
#define ATM_FX_FREQ_SLIDE 1   // ATM_SL_FRQ / ATM_SL_FRQ_ADV / ATM_SL_FRQ_OFF
#endif
#ifndef ATM_FX_ARP
#define ATM_FX_ARP 1          // ATM_ARP / ATM_ARP_OFF
#endif
#ifndef ATM_FX_NOISE
#define ATM_FX_NOISE 1        // ATM_NOISE / ATM_NOISE_OFF (retrigger)
#endif
#ifndef ATM_FX_TRANSPOSE
#define ATM_FX_TRANSPOSE 1    // ATM_ADD_TRA / ATM_SET_TRA / ATM_TRA_OFF
#endif
#ifndef ATM_FX_TREMOLO
#define ATM_FX_TREMOLO 1      // ATM_TREM / ATM_TREM_OFF
#endif
#ifndef ATM_FX_VIBRATO
#define ATM_FX_VIBRATO 1      // ATM_VIB / ATM_VIB_OFF
#endif
#ifndef ATM_FX_GLISSANDO
#define ATM_FX_GLISSANDO 1    // ATM_GLIS / ATM_GLIS_OFF
#endif
#ifndef ATM_FX_NOTE_CUT
#define ATM_FX_NOTE_CUT 1     // ATM_CUT / ATM_CUT_OFF
#endif
#ifndef ATM_FX_CUE
#define ATM_FX_CUE 1          // ATM_CUE command (stores the byte)
#endif
#ifndef ATM_FX_TEMPO
#define ATM_FX_TEMPO 1        // ATM_ADD_TEMPO / ATM_SET_TEMPO in the score
#endif
#ifndef ATM_FX_GOTO_ADV
#define ATM_FX_GOTO_ADV 1     // ATM_GOTO_ADV
#endif
#ifndef ATM_FX_STOP
#define ATM_FX_STOP 1         // ATM_STOP_CHAN
#endif

// Functions (calling one that is 0 is a compile error)
#ifndef ATM_FUNC_STOP
#define ATM_FUNC_STOP 1       // ATM.stop()
#endif
#ifndef ATM_FUNC_PAUSE
#define ATM_FUNC_PAUSE 1      // ATM.pause()
#endif
#ifndef ATM_FUNC_RESUME
#define ATM_FUNC_RESUME 1     // ATM.resume()
#endif
#ifndef ATM_FUNC_PLAYPAUSE
#define ATM_FUNC_PLAYPAUSE 1  // ATM.playPause() — needs PAUSE and RESUME
#endif
#ifndef ATM_FUNC_MUTE
#define ATM_FUNC_MUTE 1       // ATM.muteChannel()
#endif
#ifndef ATM_FUNC_UNMUTE
#define ATM_FUNC_UNMUTE 1     // ATM.unMuteChannel()
#endif
#ifndef ATM_FUNC_SFX
#define ATM_FUNC_SFX 1        // ATM.playSfx()
#endif
#ifndef ATM_FUNC_TEMPO
#define ATM_FUNC_TEMPO 1      // ATM.setTempo()
#endif
#ifndef ATM_FUNC_CUE
#define ATM_FUNC_CUE 1        // ATM.check() / ATM.check(id)
#endif

// Notes. Each flag is one octave of the 1..63 note numbers.
// C2-B2, C3-B3, C4-B4, C5-B5, C6-B6, C7-D7. Excluded notes are silent.
#ifndef ATM_OCTAVE_2
#define ATM_OCTAVE_2 1        // C2-B2
#endif
#ifndef ATM_OCTAVE_3
#define ATM_OCTAVE_3 1        // C3-B3
#endif
#ifndef ATM_OCTAVE_4
#define ATM_OCTAVE_4 1        // C4-B4
#endif
#ifndef ATM_OCTAVE_5
#define ATM_OCTAVE_5 1        // C5-B5
#endif
#ifndef ATM_OCTAVE_6
#define ATM_OCTAVE_6 1        // C6-B6
#endif
#ifndef ATM_OCTAVE_7
#define ATM_OCTAVE_7 1        // C7-D7
#endif

#endif
```

## Tracker editor

A single-file HTML tracker for this library (`atm-tracker-editor.html`; the project lives in the `trackerEditor` repository). Open it in a browser. Current release: **0.1.7**.

**Modes.** You always start in **SONG**; switch with the header buttons.

* **SONG:** four channel lanes. Patterns appear as blocks (width = tick length); unique patterns have unique colours, copies share one. Drag a pattern from the left list onto a lane to append it. Blocks pack left with no gaps; delete one (x) and the rest slide left. Click a block to enter PATTERN mode for that clip.
* **PATTERN:** one lane. **Record** writes notes and advances the cursor (hold a key to grow the tick length). **Edit** writes and plays notes but keeps the cursor (move it with Left / Right). Drag a note to move it (its length comes along; if the landing gap is shorter the note is clipped); drag the right edge to change its length.
* **SOUNDFX:** one pattern, one voice, no arrangement blocks. Choose **Play as CH0 to CH3** in the header (pulse / square / tone / noise). Export is `ATM_SFX_TRACK` for `ATM.playSfx(sfxTrack, ch)`. Switching between SONG and SFX asks for confirmation and starts from scratch.

**Load** accepts JSON and `song.h` files written with ATM commands. Macros such as `ATM_NOTE_A5_`, `ATM_SET_TRA(-5)`, `ATM_SL_VOL(-4)` are expanded. With several arrays in one file, the first valid score header is imported as four channel patterns on the song timeline. Very nested `GOTO` / `REPEAT` songs are flattened onto a 64-tick grid per channel so you can edit them.

**Export** as **commands** or **hex**: a SONG becomes `const uint8_t score[] PROGMEM` for `ATM.play(score)`, an SFX becomes `ATM_SFX_TRACK`. Rests and holds become `ATM_DELAY`; the arrangement becomes `ATM_GOTO` chains of pattern tracks.

**Playback** in the editor is Web Audio, not cycle-accurate to the AVR interrupt routine. For a device-faithful listen, compile a hex and run it in an emulator such as Ardens, or on hardware.

**Keyboard:** Space = play / stop; Left / Right = cursor in Pattern / Edit; Up / Down = piano octave; Del = clear the selected note; A-L + sharps / Z-M and Q-U = piano. Web MIDI is supported, and a computer-keyboard piano sits under the grid.

| Version | Notes |
|---|---|
| 0.1.7 | Note move / resize uses the grid (right-edge lengthen works). Sidebars scroll so they no longer overlap the piano. |
| 0.1.6 | SONG blocks, PATTERN one-lane drag and resize, Record vs Edit, SFX one channel. Load ATM command `song.h`. Export hex or commands. |
| 0.1.5 | Early SONG / SFX split. |
| 0.1.4 | Record-only hold length, compact delays. |
| 0.1.3 | Channel FX audible on Play. |
| 0.1.2 | Preview applies SET VOLUME + SLIDE VOLUME per tick; hold notes; piano under grid; channel / step FX. |
| 0.1.0 | First HTML editor (replacing the old TEAM ARG React trackerEditor). |

---

# Song format and songwriting guide

**Audience:** a person or an AI starting from zero. Every statement here was checked against the ATMlib source (`ATMcmds.h`, `ATMlibimpl.h`) and against songs that were tested on hardware (Italo Night, Escaper Droid, Midnight Run, Quest Theme, Star Light). Where something is an empirical finding rather than source code, it says so.

**How to use this part:** give an AI assistant this README plus the `ATMlib` folder, and say what you want (style, key, length, byte budget, loop or one-shot). It should read sections 0, 4, 6 and 9 first.

**Loops:** all loops use the *self-loop* method of section 6 (no `ATM_STOP_CHAN`, no `ATM_GOTO_ADV`). The older `STOP` + `GOTO_ADV` loop leaves one silent tick at the seam, audible as a "hop"; the self-loop fixed it (confirmed by ear). `extras/tools/convert.py` converts old songs.

---

## 0. Start here

### 0.1 House rules for songs in this project (always apply)

1. **Double the tempo.** Wherever an emulator or offline analysis says `17`, write `ATM_SET_TEMPO(34)`. On the real Arduboy the song plays at about *tempo-value* ticks per second (see 5.1). Quest Theme uses 34.
2. **Deliver only the song `.h`.** No WAV file unless asked.
3. **Write songs as commands**, e.g. `ATM_NOTE_A4, ATM_DELAY(8), ATM_VOL(38)`. Never write track bodies as raw hex. The small header (track count, track addresses, channel entry tracks) is hex, exactly as in the reference `song.h`.
4. **Byte budget means the whole array**, including the header. "Max 250 bytes" = `sizeof(song) <= 250`.
5. **Loops must be hop-free.** A `STOP` + `GOTO_ADV` restart silences all four channels for one tick (about 30 ms) at every loop, and it is clearly audible. Make every loop a *self-loop*: each channel's entry track ends with `ATM_GOTO(<itself>)` and every channel sums to exactly `L` ticks. See section 6.
6. **Tempo belongs to the sketch when the author says so.** If the sketch sets it from the sketch (`ATM.setTempo()`), leave `ATM_SET_TEMPO` out of the song (`quest.h` has none now: without it the song plays at ATMlib's default of 25, not 34).
7. **Quality bar:** Italo Night, Midnight Run and Quest Theme are the reference songs; the in-game loops (section 8.7) are the reference for tiny budgets. Pixel Chase was rejected: too plain, and actually broken (section 12).
8. **Game music at 60 %.** Songs meant for in-game use with sound effects are written (or scaled) to about 60 % of the volume of title music, with `ATM_VOL(MV(n))` and `ATM_SL_VOL(MS(n))` (see *Sound effects*). Title music stays at 100 %.

### 0.2 Starter prompt for a fresh session

> Read `ATMlib/README.md` completely (especially the songwriting guide, sections 0 to 13), then `ATMlib/src/ATMcmds.h` and one example `song.h`. Write a new ATMlib song: *(style, mood, key, number of bars or length, loop or one-shot, byte budget)*. Follow section 0.3, verify with the simulator, compile-check the header, and deliver only the `.h` in `ATM_*` commands with the doubled tempo.

### 0.3 The workflow in ten lines

1. Read `ATMcmds.h` for the real opcode values. Never guess them (section 12, mistake 1).
2. Decide: tempo (doubled), key, number of bars, loop or one-shot, byte budget.
3. Time grid: **16 ticks = 1 beat, 64 ticks = 1 bar** (used by all approved songs).
4. Plan every channel in ticks. For a loop of `L` ticks every channel must contain exactly `L` ticks of delay, and its entry track must end with `ATM_GOTO(itself)` (section 6). No `STOP`, no `GOTO_ADV`, no trimmed last delay, no `_last` tracks.
5. Write the song with a **generator script** (named tracks, automatic track numbers and offsets). `extras/tools/gen_quest.py` is a complete example.
6. Run the **simulator** (`extras/tools/atmsim.py`): no channel ever stops (except an unused one at tick 0), the song never restarts, the per-tick timeline repeats exactly with period `L`, call depth is at most 7, no note out of range.
7. Print the notes per bar and check harmony and register by eye.
8. Emit the `.h` with `ATM_*` macros and **compile it against the real `ATMcmds.h`**, then compare the compiled bytes with the generator's bytes (`extras/tools/check_header.cpp`).
9. Check the byte budget.
10. Deliver the `.h` only, with a short summary (form, channels, size).

---

## 1. Mental model

* ATMlib is a 4-channel tracker-style music player for the Arduboy. Default voices: **CH0 PULSE, CH1 SQUARE, CH2 SAW, CH3 NOISE**. Waveforms are fixed at compile time (`ATMconfig.h`), not per song.
* Time is counted in **ticks**. Tempo sets the tick rate.
* A song is a set of **tracks** (byte sequences, like subroutines). Each channel starts at one *entry track* and runs commands one after another. Tracks can call other tracks (`GOTO` = call and return, `REPEAT` = call n+1 times).
* Almost everything is **state that persists** until you change it: volume, volume slide, transposition, arpeggio. This is the source of most bugs and most savings.
* A note plays until the next note or the next `VOL(0)`. A `DELAY` makes the channel wait; the sound keeps going meanwhile.

---

## 2. Song data format

### 2.1 Layout


|**Section**					| **Field**					| **Type**			| **Description**
|---							| ---						| ----------------	| ---
|**Track table**				|							|					| **Number of tracks and their addresses**
|								| Track count				| UBYTE (8-bits)	| Number of tracks in the file/array
|								| Address track 1			| UWORD (16-bits)	| Location in the file/array for track 1
|								| …							| …					| …
|								| Address track *__N__*		| UWORD (16-bits)	| Location in the file/array for track *__N__ (0 … 255)*
|**Channel entry tracks**		|							|					| **For each channel, track to start with**
|								| Channel 0 track			| UBYTE (8-bits)	| Starting track index for channel 0
|								| … 						| …					| …
|								| Channel 3 track			| UBYTE (8-bits)	| Starting track index for channel 3
|**Track 0**					|							|					| **Commands and parameters for track 0**
|								| Command 0					| UBYTE (8-bits)	| See command list
|								| *and its* Parameters		| none/variable		| *See __parameter list__ for each command*
|								| …							| …					| …
|								| Command N					| UBYTE (8-bits)	|
|								| *and its* Parameters		| none/variable		|
|**…**							| **…**						| **…**				| **…**
|**Track _N_**					|							|					| **Commands and parameters for track _N_** *(0-255)*

### 2.2 Byte layout and rules

```
byte 0               N = number of tracks
bytes 1 .. 2N        N x uint16 little-endian: offset of each track,
                     relative to the first byte AFTER the 4 channel-entry bytes
next 4 bytes         entry track number for CH0, CH1, CH2, CH3
then                 track data, tracks back to back, in table order
```

* **Header cost = 1 + 2N + 4 bytes.** Every extra track costs 2 bytes of table plus its closing `RETURN`. The 250-byte budgets are tight mostly because of this.
* Tracks are addressed by index 0..N-1 (max 255 tracks).
* A *called track* ends with `ATM_RETURN`. An *entry track* of a **looping** song ends with `ATM_GOTO(<itself>)` (section 6). An entry track of a **one-shot** song ends with `ATM_STOP_CHAN`. An *unused channel* points at a track that holds only `ATM_STOP_CHAN` (1 byte).
* The song array must be `const uint8_t PROGMEM`.

---

## 3. Command reference

Values are from `ATMcmds.h` (the source of truth). Sections 3.1 and 3.3 are the byte-level spec (opcode ranges and effect numbers, with the bytes each one uses); 3.2 and 3.4 are the practical macro reference.

### 3.1 Opcode map


|**Command (_X_)**					| **Parameter**			| **Type**				| **Description** | **Bytes**
|---               	   				| ---					| ------------------	| --- | ---
|**0<br/>0x00**						|						|						| Silence (note off) | 1
|**1…63<br/>0x00+__X__**			| note *(__X__)*		| UBYTE (8-bits)		| Start playing note *[__X__]* where 1 is a C2 (see section 5.2).<br/>See [Frequency to Tone](extras/frequencyToTone.md "Frequency to Tone table")<br/>**_Note:_** everytime a note is played, volume is re-triggered | 1
|**64…159<br/>0x40…0x9F**			|						|						| Configure effects (fx) | 1 + params
|									| *See section 3.3*		| none/variable			| Effect is *[__X__ - 64]* | 
|**160…223<br/>0x9F+__t__**			| Ticks (*__t__*)		| UBYTE (8-bits)		| Delay for *[__X__ - 159]* or *[__t__]* ticks<br/>**_Note:_** delay of 0 does not exist and maximum is 64 ticks | 1
|**224, __Y__<br/>0xE0, __Y__**		| Ticks (*__Y__*)		| VLE (8/16-bits)		| Long delay for *[__Y__ + 65]* ticks<br/> **_Note:_** LONG delay starts at 1 higher than SHORT delay | 2
|**252, N<br/>0xFC, N**				| Track *__N__*			| UBYTE (8-bits)		| Call/run/goto specified track<br/>Track index where *__N__* is the number of the track to go to | 2
|**253, Y, N<br/>0xFD, Y, N**		|						|						| Repeated call/run/goto specified track | 3
|                  	   				| Loop count (*__Y__*)	| UBYTE (8-bits)		| Repeat *[__Y__ + 1]* times (total) | 
|                  	   				| Track  *__N__*		| UBYTE (8-bits)		| Track index where *__N__* is the number of the track to go to | 
|**254<br/>0xFE**					|						|						| Return/end of track marker | 1
|**~~225…255~~**		  	   		|						|						| ~~RESERVED~~ | 

### 3.2 Core macros

| Macro | Bytes | Meaning |
|---|---|---|
| `ATM_NOTE_C2` .. `ATM_NOTE_D7` | 1 byte, value 1..63 | Start a note. C2=1, C#2=2 ... B2=12, C3=13, A3=22, A4=34, A5=46, C6=49, D7=63. Sharps use an **underscore**: `ATM_NOTE_F5_` is F#5. |
| raw `0x00` | 1 byte | Silence (note off). There is **no** `ATM_NOTE_OFF` macro. Often `ATM_VOL(0)` is used instead. |
| `ATM_DELAY(n)` | `0x9F+n` | Wait n ticks, **n = 1..64**. `ATM_DELAY(0)` is `0x9F` = STOP, so never use 0. |
| two delays in a row | | Valid and seamless: `ATM_DELAY(64), ATM_DELAY(10)` = 74 ticks. |
| `ATM_LONG_DELAY(ticks)` | `0xE0, ticks-65` | Wait **65 to 192 ticks** in 2 bytes (the byte after `0xE0` is a variable-length number, 7 bits per byte, the delay is that number + 65). For other lengths chain `ATM_DELAY`s: `ATM_LONG_DELAY(100), ATM_DELAY(30)` = 130 ticks. Pass the real tick count; do not pass 0 to 64 (the subtraction would wrap). |
| `ATM_GOTO(t)` | `0xFC, t` | Call track t; continue after it returns. |
| `ATM_REPEAT(n, t)` | `0xFD, n, t` | Call track t **n+1 times** in total. `REPEAT(3,t)` = 4 runs. |
| `ATM_RETURN` | `0xFE` | End of a called track. |
| `ATM_STOP_CHAN` | `0x9F` | Stop this channel **and set its volume to 0** (4.4). Use it for one-shots and unused channels. **Never use it to loop**: the restart leaves one silent tick. |

**Reserved range.** Command bytes `225..255` are reserved and ignored. The former `0xFF` "embedded data" command (length + data bytes for the host) has been **removed** from the library; use `ATM_CUE(v)` (`0x57, v`, effect 23, read with `ATM.check()`) to signal the sketch instead. The Bytes columns match the `// uses N bytes` comments in `ATMcmds.h`.

### 3.3 Effect numbers (full detail)

Every effect is `64 + n`, i.e. `0x40 + n`, followed by its parameter bytes.


|**Effect**					| **Parameter**											| **Type**							| **Description** | **Bytes**
|---							| ------------------------								| ---------------------------		| --- | ---
|**64+0<br/>64<br/>0x40**	| set Volume (*__X__*)									| UBYTE (8-bit) 					| Set volume to *[__X__]*.<br/>**_Note:_** If the combined volume of all channels<br/>exceed 255 there may be rollover distortion. This<br/>should not be disallowed, as it may be usesful as<br/>an effects hack for the musician. There should<br/>however be a non-interfering warning when a<br/>musician enters a value above 63 for ch 1-3 or 32<br/>for ch 4 (noise). ch 4 the volume is counted double,<br/>so 32 is actually 64. | 2
|**64+1<br/>65<br/>0x41**	| slide Volume ON (*__X__*)								| UBYTE (8-bit) 					| Slide the volume with an amount (positive or<br/>negative) of *[__X__]* for every tick.<br/>**_Note:_** This results in a fade-in or fade-out<br/>effect. There should be a non-interfering warning<br/>when sliding would result in exceeding 63 for<br/>ch 1-3 and 32 for ch 4. | 2
|**64+2<br/>66<br/>0x42**	| slide Volume ON<br/>advanced (*__X__*) (*__Y__*)		| UBYTE (8-bit)<br/>UBYTE (8-bit)	| Slide the volume with an amount (positive or<br/>negative) of *[__X__]* for every [*__Y__*] ticks.<br/>*[__Y__]* includes 2 parameters: RRtttttt<br/>R = reserved and t = ticks. | 3
|**64+3<br/>67<br/>0x43**	| slide Volume OFF										|  									| Stops the volume slide | 1
|**64+4<br/>68<br/>0x44**	| slide Frequency ON (*__X__*)							| UBYTE (8-bit)						| Slide the frequency with an amount (positive or<br/>negative) of *[__X__]* for every tick. <br/>**_Note:_** The amount of slide is limited<br/>between -127 to 127 | 2
|**64+5<br/>69<br/>0x45**	| slide Frequency ON<br/>advanced (*__X__*) (*__Y__*)	| UBYTE (8-bit)<br/>UBYTE (8-bit)	| Slide the frequency with an amount (positive or<br/>negative) of *[__X__]* for every [*__Y__*] ticks.<br/>*[__Y__]* includes 2 parameters: RRtttttt<br/>R = reserved and t = ticks. | 3
|**64+6<br/>70<br/>0x46**	| slide Frequency OFF									|  									| Stops the frequency slide | 1
|**64+7<br/>71<br/>0x47**	| set Arpeggio (*__X__*)(*__Y__*)						| UBYTE (8-bit)<br/>UBYTE (8-bit)	| Next to the current playing note, play a second<br/>and third note *[__X__]* for every *[__Y__]* ticks.<br/>*[__X__]* includes 2 parameters: AAAABBBB, where<br/>AAAA = base + amount to second note and<br/>BBBB = second note + amount to third note.<br/>*[__Y__]* includes 4 parameters: FEDttttt,<br/>where F = reserved, E = toggle no third note,<br/>D = toggle retrigger, ttttt = tick amount.<br/>**_Note:_** Arpeggio is used for playing 3 notes<br/>out of a chord indivually | 3
|**64+8<br/>72<br/>0x48**	| Arpeggio OFF											|									| Stops the arpeggio | 1
|**64+9<br/>73<br/>0x49**	| SET Retriggering noise ON (*__X__*)					| UBYTE (8-bit)						| Noise channel consists of white noise. By setting<br/>retriggering *[__X__]* it swithes the entrypoint at<br/>a given speed. *[__X__]*  includes 2 parameters:<br/>AAAAAABB , where AAAAAA = entry point and<br/>BB = speed (0 = fastest, 1 = faster , 2 = fast) | 2
|**64+10<br/>74<br/>0x4A**	| Retriggering noise OFF								|									| Stops the retriggering for the noise on channel 3 | 1
|**64+11<br/>75<br/>0x4B**	| add Transposition (*__X__*)							| UBYTE (8-bit)						| Shifts the played notes by adding *[__X__]* to<br/>the existing transposition for all playing notes.<br/>**_Note:_** The amount of shift is limited<br/>between -127 to 127. However there should be<br/>a non-interfering warning when transposing would<br/>result in exceeding 63 or get lower than 0 | 2
|**64+12<br/>76<br/>0x4C**	| set Transposition (*__X__*)							| UBYTE (8-bit)						| Shifts the played notes by setting the transposition<br/>to [__X__]* for all playing notes.<br/>**_Note:_** The amount of shift is limited<br/>between -127 to 127. However there should be a<br/>non-interfering warning when transposing would<br/>result in exceeding 63 or get lower than 0 | 2
|**64+13<br/>77<br/>0x4D**	| Transposition OFF										|									| Stops the transposition | 1
|**64+14<br/>78<br/>0x4E**	| set Tremolo (*__X__*)(*__Y__*)						| UBYTE (8-bit)<br/>UBYTE (8-bit)	| *[__X__]* sets Depth.<br/>*[__Y__]* includes 4 parameters<br/>RxxBBBBB R = Retrig, x = reserved , B = rate<br/>**_Note:_** Tremolo and Vibrato can **NOT**<br/>be combined in the same stack | 3
|**64+15<br/>79<br/>0x4F**	| Tremolo OFF											|									| Stops the tremolo | 1
|**64+16<br/>80<br/>0x50**	| set Vibrato (*__X__*)(*__Y__*)						| UBYTE (8-bit)<br/>UBYTE (8-bit)	| *[__X__]* sets Depth.<br/>*[__Y__]* includes 4 parameters<br/>RxxBBBBB R = Retrig, x = reserved , B = rate<br/>**_Note:_** Tremolo and Vibrato can **NOT**<br/>be combined in the same stack | 3
|**64+17<br/>81<br/>0x51**	| Vibrato OFF											|									| Stops the vibrato | 1
|**64+18<br/>82<br/>0x52**	| SET Glissando (*__X__*)								| UBYTE (8-bit)						| *[__X__]* includes 2 parameters: Vttttttt<br/>V = value ( 0 = go 1 note up, 1 = go 1 note down)<br/>and t = amount of ticks, between each step | 2
|**64+19<br/>83<br/>0x53**	| Glissando OFF											|									| Stops the Glissando | 1
|**64+20<br/>84<br/>0x54**	| SET Note Cut (*__X__*)								| UBYTE (8-bit)						| *[__X__]* sets the equal amount of ticks<br/>between note ON and OFF | 2
|**64+21<br/>85<br/>0x55**	| Note Cut OFF											|									| Stops the Note Cut | 1
|**64+22<br/>86<br/>0x56**	| *unused*												| 											| Not implemented by the playroutine (there is no `ATM_WAVEFORM`). Waveforms are chosen at compile time in `ATMconfig.h`. | 
|**64+23<br/>87<br/>0x57**	| CUE (*__X__*)											| UBYTE (8-bit)						| Signals the sketch: stores byte *[__X__]* which the sketch reads with `ATM.check()` (returns it once, then 0).<br/>Use `ATM_CUE(x)` to sync game events with the music | 2
|**…**						| **…**													| **…**								| **…** | 
|**64+92<br/>156<br/>0x9C**	| ADD song tempo (*__X__*)								| UBYTE (8-bit)						| adds *[__X__]* to the tempo of the song.<br/>Total value should be between 0 - 127<br/>**_Note:_** the higher the tempo to more CPU it takes. | 2
|**64+93<br/>157<br/>0x9D**	| SET song tempo (*__X__*)								| UBYTE (8-bit)						| (re-)sets *[__X__]* as the tempo of the song.<br/>Standard is 25. Value should be between 0 - 127<br/>**_Note:_** the higher the tempo to more CPU it takes. | 2
|**64+94<br/>158<br/>0x9E**	| GOTO advanced<br/><br/>(*__W__*)<br/>(*__X__*)<br/>(*__Y__*)<br/>(*__Z__*)	| <br/><br/>UBYTE (8-bit)<br/>UBYTE (8-bit)<br/>UBYTE (8-bit)<br/>UBYTE (8-bit)	| **_Note:_** handy command for having an intro<br/>and a repeating song part<br/>For channel __0__ go to track __W__<br/>For channel __1__ go to track __X__<br/>For channel __2__ go to track __Y__<br/>For channel __3__ go to track __Z__ | 5
|**64+95<br/>159<br/>0x9F**	| STOP current channel									|									| channel is no longer being processed<br/>**_Note:_** if all channels have reached STOP, the song ends | 1

#### Thoughts on effects:

**Note:** These are the primitives to be implemented in the playroutine effects processor. Most will have several effect command numbers associated with them for various aspects of the same primitive. Effects can be combined but not stacked, but some combinations may have undesired/interesting interference.

* Volume slide: a gradual increasing or decreasing of the volume.
* Frequency slide: a gradual increasing or decreasing of the [frequency](https://en.wikipedia.org/wiki/Frequency "frequency wikipedia").
* Arpeggio: a group of [notes](https://en.wikipedia.org/wiki/Musical_note "note wikipedia") which are rapidly and automatically played one after the other.
* Retriggering (on [note](https://en.wikipedia.org/wiki/Musical_note "note wikipedia") or by automation): oscillators are restarted either automatically or at the start of each new note.
* Transposition (also for microtonals): play [notes](https://en.wikipedia.org/wiki/Musical_note "note wikipedia") in a different key, or fine tune notes to provide microtonals; frequencies that are in between notes.
* Tremolo: a slight, rapid and regular fluctuation in the amplitude/volume of a [note](https://en.wikipedia.org/wiki/Musical_note "note wikipedia").
* Vibrato: a slight, rapid and regular fluctuation in the [pitch](https://en.wikipedia.org/wiki/Pitch_(music) "pitch wikipedia") of a [note](https://en.wikipedia.org/wiki/Musical_note "note wikipedia").
* Glissando: controls if and how a gradual frequency slide "snaps" to adjacent notes.
* Note cut (with delay and automation): provides a method to stutter and adjust note timing.

### 3.4 Effect macros (all are `64+n`, i.e. `0x40+n`)

| Macro | Bytes | Meaning |
|---|---|---|
| `ATM_VOL(v)` | `0x40, v` | Set volume 0..63. Also stores v as the volume re-applied at every later note-on (see 4.2). |
| `ATM_SL_VOL(s)` | `0x41, s` | Add signed s to the volume **every tick** (clamped at 0 only, see 4.9: a positive slide is NOT clamped at 63). A negative value gives a decay after each note. |
| `ATM_SL_VOL_ADV(s, t)` | `0x42, s, t` | Slide s every t+1 ticks (`t` is the low 6 bits; bit 7 = no clamping at all). Used for long fades (4.9). **Caution:** with this form the volume is *not* re-applied at note-on (4.2). |
| `ATM_SL_VOL_OFF` | `0x43` | Stop the volume slide. |
| `ATM_SL_FRQ(s)` / `_ADV` / `_OFF` | `0x44` / `0x45` / `0x46` | Frequency slide (pitch drop for kicks: `-127`). Also disables note-on volume re-apply. |
| `ATM_ARP(a, y)` | `0x47, a, y` | Arpeggio, see 4.6. |
| `ATM_ARP_OFF` | `0x48` | Arpeggio off. |
| `ATM_NOISE(x)` / `ATM_NOISE_OFF` | `0x49, x` / `0x4A` | Noise retrigger: `x = AAAAAABB` entry point + speed (0 fastest). CH3 only. |
| `ATM_ADD_TRA(d)` | `0x4B, d` | Add signed d to the transposition. |
| `ATM_SET_TRA(v)` | `0x4C, v` | Set the transposition to signed v (semitones). |
| `ATM_TRA_OFF` | `0x4D` | Transposition = 0. |
| `ATM_TREM(depth, rate)` / `_OFF` | `0x4E, d, r` / `0x4F` | Tremolo: volume moves by `depth` every tick, reversing every rate+1 ticks (triangle). Shares state with vibrato, so one of the two per channel. |
| `ATM_VIB(depth, rate)` / `_OFF` | `0x50, d, r` / `0x51` | Vibrato: frequency (Hz) moves by `depth` every tick, reversing every rate+1 ticks. `VIB(3,3)` is about +-24 cents at 880 Hz. |
| `ATM_GLIS(x)` / `_OFF` | `0x52, x` / `0x53` | Glissando; bit 7 = direction (1 = down), low 7 bits = ticks per semitone step. |
| `ATM_CUT(n)` / `_OFF` | `0x54, n` / `0x55` | Staccato **gate**: sound for n+1 ticks, silence for n+1 ticks, repeating. Use `0x20+n` so every note-on restarts it. Shares state with the arpeggio (not both on one channel) and **only works with transposition = 0** (4.9). |
| `ATM_CUE(v)` | `0x57, v` | Hands a byte to the sketch (`ATM.check()`). |
| `ATM_ADD_TEMPO(v)` | `0x9C, v` | Add to the tempo (the tempo is a byte, so `(uint8_t)-1` subtracts 1). Ritardando: one `ADD_TEMPO(-1)` per beat. |
| `ATM_SET_TEMPO(v)` | `0x9D, v` | Set the tempo, 0..127, default 25. |
| `ATM_GOTO_ADV(a,b,c,d)` | `0x9E, a,b,c,d` | Set the **restart track** for CH0..CH3 used when all channels have stopped (4.4). **Obsolete for loops** (it needs `STOP`, which leaves a silent tick); do not use it any more. |

### 3.5 Writing rules for the `.h`

* Negative arguments need a cast: `ATM_SL_VOL((uint8_t)-5)`, `ATM_ADD_TRA((uint8_t)-4)`. Plain `-5` fails strict C++ narrowing.
* Sharps: `ATM_NOTE_F5_`, never `ATM_NOTE_F#5`.
* Use the macro for every note and delay; comment each track.

---

## 4. How the playroutine really behaves

All of this is from `ATMlibimpl.h`. A Python model of it is `extras/tools/atmsim.py`.

### 4.1 Tick order and delay

Each tick, for channel 0, 1, 2, 3 in that order: apply effects (slides, arpeggio, ...), then, if the channel's delay is 0, run commands until one sets a delay. `ATM_DELAY(d)` makes the note/state last exactly **d ticks**. When a delay expires, the next command runs in that same tick, so chained delays and calls cost no extra tick.

### 4.2 Notes, volume and transposition

* A note value 1..63 sets the pitch to `note + transposition` **at the moment of the note-on**. Changing the transposition later does not move a sounding note.
* **Keep `note + transposition` inside 1..63.** With all octaves compiled in, the pitch table is read without bounds checking, so out-of-range values read garbage.
* At every note-on the volume is reset to the last `ATM_VOL(v)` value, **but only when the active slide is a plain `ATM_SL_VOL`/none** (config byte 0). With `SL_VOL_ADV` or any frequency slide active the volume is not reset. This is what makes "VOL(38) + SL_VOL(-2)" a plucky lead: every note starts at 38 and decays by 2 per tick.
* Volume range 0..63 per channel. The noise channel's volume is halved in the mixer. Very loud sums of all channels can distort.

### 4.3 Calls, the call stack and the track-0 trap

* `GOTO`/`REPEAT` push a return frame (counter, track, position). Maximum depth **7**.
* A call to the track the channel is *currently in* does **not** push; it just restarts that track, **in the same tick**, without touching volume or any other state. This is what the self-loop of section 6 is built on. (Because every channel starts with "current track = 0", the very first `GOTO(entry)` of a channel whose entry track is not 0 does push one frame, once; count it in the depth limit.)
* **Every channel starts with "current track = 0", whatever its entry track is.** So a channel that calls track 0 from its entry track does not push a frame. When track 0 then returns with an empty stack, it simply restarts itself forever.
  **Rule: never call track 0 from another track.** Track 0 may be used only as a channel entry (CH3 in Quest Theme) or as a dummy that nobody calls (the older songs have a "silent" track 0). An entry track 0 that ends with `ATM_GOTO(0)` is fine: that is a call to the track it is already in (see below).
  *Measured example:* the rejected `pixelchase.h` had its melody bar as track 0. In the simulator CH0 plays bars 1-6 correctly, then repeats bar 1 forever and never stops, while the other three channels stop at tick 511 and the song never restarts.
* `RETURN` with a repeat counter above 0, or with an empty stack, **restarts the current track** instead of leaving it. So ending an entry track with `RETURN` instead of `STOP` loops that track forever.
* `REPEAT(n,t)` runs t n+1 times. Nested repeats are fine (counters are saved in the frame).

### 4.4 Stop, end of song and looping

* `STOP` **sets the channel volume to 0** (`ch->vol = 0`, `ATMlibimpl.h`, case 95) and parks the channel with an endless delay. When all four channels have stopped:
  * if the sum of the four `GOTO_ADV` restart tracks is 0 (or `GOTO_ADV` was never executed) the song **ends**;
  * otherwise every channel jumps to its restart track and the song **continues**.
* **This restart is the cause of the loop "hop".** The channels that stopped first are silent (volume 0) from their `STOP` until the last one stops, and the restart itself costs one more tick, so every loop contains a tick in which *all four channels are silent*. The period is exactly right, but the seam is audible (about 30 ms at tempo 34). This was found by ear on Italo Night, Escaper Droid, Neon Heart, Quest Theme, Star Light and the first version of the in-game songs.
* **Looping songs therefore must not stop at all.** Loop with a self-`GOTO` at the end of every channel's entry track (section 6). Do not use `STOP`, do not use `GOTO_ADV`.
* **One-shot songs** end with `STOP` on every channel and contain no `GOTO_ADV` (Midnight Run, a 3-minute one-shot, has none. When Claude once left it in, the song restarted).
* *Background, only for old files:* the restart is decided right after the last channel stops, inside the channel loop. If the last channel to stop is not CH3, the channels after it start the new loop one tick earlier than the ones before it, so all four channels had to stop on the same tick. The restart does not reset volume, slides, arpeggio or transposition. Both facts are irrelevant for a self-loop, but the second one still holds: **state persists across the loop**, so set what you need at the start of each channel's main track (typically `ATM_SET_TRA(0)`).

### 4.5 Slides

`SL_VOL(s)` changes volume by s every tick. `SL_FRQ(-127)` on a square note makes the classic pitch-drop kick (Italo Night). After `SL_FRQ` the volume is not re-applied at note-on, so set `ATM_VOL` explicitly before the note.

### 4.6 Arpeggio

`ATM_ARP(a, y)` plays three notes in rotation from the note you start:

* `a = AAAABBBB`: second note = base + A semitones; third = second + B.
* `y = FEDttttt`: `D` (0x20) = restart the rotation at every note-on; `E` (0x40) = no third note (two-note arp); `ttttt` = ticks per step minus one. `0x20` means one tick per step (about 11 rotations per second at tempo 34); `0x21` two ticks, etc.
* Chord values (README `extras/notesInChords.md` + inversions):

| Chord | `a` | Notes from base |
|---|---|---|
| major, root position | `0x43` | root, +4, +7 |
| minor, root position | `0x34` | root, +3, +7 |
| major, 1st inversion (3rd in bass) | `0x35` | e.g. A C F = F/A |
| major, 2nd inversion (5th in bass) | `0x54` | e.g. G C E = C/G |
| minor, 1st inversion | `0x45` | e.g. C E A = Am/C |
| minor, 2nd inversion | `0x53` | e.g. E A C = Am/E |
| maj7 colour (no 5th) | `0x47` | root, +4, +11 (used in Escaper Droid) |
| m7 colour (no 5th) | `0x37` | root, +3, +10 |

* **Inversions keep the stabs in one register.** Italo Night plays every chord from around G3..B3 using `0x34 / 0x35 / 0x54` with a changing base note.
* **Do not combine `ATM_ARP` with transposition on the same channel.** During arpeggio steps the playroutine adds the transposition a second time (`ATMlibimpl.h`, the line `ATM__FREQ(arpNote + ch->transConfig)` while `arpNote` already includes it). Put the arpeggio on one channel with explicit root notes, and use transposition on other channels.
* `ATM_NOTE_x` after `ATM_ARP` sets the base note; a new note restarts the rotation when `0x20` is set.

### 4.7 Noise channel (CH3)

CH3 is noise; drums are made purely with `VOL` + `SL_VOL` + `DELAY` (no notes needed). Volume is halved in the mixer. See section 8.1 for the approved recipes.

### 4.8 Known traps in the library files

* `ATM_DELAY(0)` is the STOP byte.
* Note 1 is **C2** (`ATM_NOTE_C2`). Older documents called it C1 or C4: the name is a label, see section 5.2.
* README says tempo values are 0..127 and "the higher the tempo, the more CPU".

### 4.9 Effect details verified in `ATMlibimpl.h` (used by Cutlass Cove)

Per tick, per channel, the order is: noise retrigger, glissando, volume/frequency slide, arpeggio/cut, tremolo/vibrato, then the command stream.

* **One slide slot per channel.** `SL_VOL`, `SL_VOL_ADV`, `SL_FRQ`, `SL_FRQ_ADV` share a single slide. Starting one replaces the other. Cannon boom on the bass: `ATM_SL_FRQ(-24)` for 16 ticks, then `ATM_SL_VOL(-3)` both ends the pitch slide and starts the volume decay.
* **Positive volume slides are not clamped at 63** (a dangling `else` in the library: only freq is clamped above). `vol` is a byte, so a fade-in must be **stopped on purpose** (`ATM_SL_VOL_OFF`, or a new `ATM_SL_VOL`) before it reaches 63. Negative slides clamp at 0. Same for tremolo.
* **Fades:** `ATM_VOL(0), ATM_SL_VOL_ADV(1, 8)` adds +1 every 9 ticks and, because the config byte is non-zero, notes do not reset the volume, so the fade runs straight through note-ons. Fade-out: `ATM_SL_VOL_ADV((uint8_t)-1, 5)` from a starting `ATM_VOL`. `SL_VOL_ADV` and `SL_FRQ` set the config byte, so afterwards use plain `ATM_SL_VOL(x)` (which sets config 0) before expecting note-on volume resets again.
* **Frequency slide** `SL_FRQ(s)` moves the frequency by s **Hz per tick** (table of `extras/frequencyToTone.md`); clamped 0..9397.
* **Tremolo** is a triangle LFO on the volume: the first half goes *down*. `ATM_TREM(1, 15)` = sea swell with a 32-tick period. `ATM_TREM(6, 0)` on a rising volume (`VOL(10), SL_VOL(2)`) = a snare roll in 8 bytes; end it with `ATM_TREM_OFF`.
* **Vibrato** is the same on the frequency. A new note restarts the frequency, the LFO phase carries on, so the vibrato stays bounded.
* **Glissando** `ATM_GLIS(x)`: moves the note by one semitone every `(x & 0x7F)+1` ticks, upward, or downward when bit 7 is set (`0x82` = down, 3 ticks per step); stops at 1 and 63. End it with `ATM_GLIS_OFF`. **Loop trap:** the glissando step counter is not reset by `ATM_GLIS_OFF` or by a new note, it just freezes. If the glissando runs for a number of ticks that is not a multiple of its period `(x & 0x7F)+1`, the next run starts at a different phase and every loop pass shifts by a tick or more (found in Orbital Rush: `GLIS(0x02)` = period 3 over 64 ticks). Pick a period that divides the run length (`GLIS(0x01)`, `0x03`, `0x07` over 64 ticks). The same holds for the vibrato/tremolo period (section 6). Check with the simulator that pass 1 and pass 2 are identical tick by tick.
* **Note cut is a repeating gate:** after the note-on it sounds for n+1 ticks, is silent for n+1 ticks, then sounds again. With `ATM_CUT(0x23)` and 8-tick notes you get 4 ticks on, 4 off, exactly staccato. Notes longer than 2*(n+1) ticks come back to life, so keep the notes at one length. **With a non-zero transposition the "silence" plays note number = transposition** (the table is read at index 0+tra, negative = garbage), so use `CUT` only where transposition is 0.
* **Noise retrigger** `ATM_NOISE(x)` (x = entry point * 4 + speed; `0x14` = point 5, speed 0) reseeds the LFSR every tick = low rumble (thunder). It shares `reCount` with the volume buffer, so use it on CH3 where no notes are played, and call `ATM_NOISE_OFF` afterwards.
* **Cue:** `ATM_CUE(v)` stores one byte; `ATM.check()` returns it and clears it, `ATM.check(id)` tests without clearing. A cue is overwritten by the next one if the sketch does not poll often enough. Put cues on a track that is already part of the beat sequence (2 bytes per marker).
* **Tempo changes** are global (any channel); put them on CH3 so the tick domain stays easy to read. Ticks are all that other channels count, so a tempo change never breaks the alignment.

---

## 5. Timing, tempo and pitch

### 5.1 Tempo

* Default tempo (no `SET_TEMPO`) is 25. `cia = 15625 / tempo`.
* **Empirical rule (measured on real hardware):** ticks per second ≈ the tempo value. An offline emulator that gives 2 ticks per second per tempo unit plays twice as fast as the hardware. Hence the house rule: **double the emulator's tempo value**.
* With 16 ticks per beat: **BPM = tempo x 3.75.** Examples: 24 -> 90, 30 -> 112.5, 32 -> 120, **34 -> 127.5**, 36 -> 135, 40 -> 150.
* Song length in seconds ≈ ticks / tempo. Quest Theme: 1024 ticks / 34 ≈ 30 s.

### 5.2 Pitch

* The note names are not concert pitch. Working assumption used in all approved songs (stated in the Italo Night header): the table frequency is halved by the 31250 Hz mixing, so a note sounds about **an octave above its name and about 80 cents flat** (`ATM_NOTE_A2` ≈ 210 Hz).
* What matters is relative pitch and register. Registers (by label) that worked: lead A4..D6, chord arps G3..E4, bass A2..A3 (plus transposition), drums on the noise channel.

---

## 6. Hop-free loops: the self-loop (the most important section)

Let the loop be `L` ticks long (Quest Theme: 16 bars x 64 = 1024).

**The method**

1. Every channel's entry track ends with **`ATM_GOTO(<that same track>)`** instead of `ATM_STOP_CHAN`. A call to the track you are already in does not push the call stack: it jumps back to the first command of the track **in the same tick**. No tick is lost, nothing is silenced, volume and slides simply carry on.
2. Every channel contains **exactly `L` ticks of delay** (not `L-1`). The last note of the last bar keeps its full delay.
3. No `ATM_STOP_CHAN`, no `ATM_GOTO_ADV`, no trimmed delay, no `_last` variant tracks.
4. An unused channel's entry track holds a single `ATM_STOP_CHAN` (1 byte); it is silent from tick 0, which is what you want.
5. State persists across the loop (volume, slide, transposition, arpeggio). Set what the pass depends on at the start of each entry track (typically `ATM_SET_TRA(0)`), and let the state at the end equal the state at the start.

```
Song x[] = {
  ...
  //"Track 3" lead
  ATM_VOL(38),
  ATM_SL_VOL((uint8_t)-2),
  ...bars...
  ATM_GOTO(3),   // loop: restart this track (no STOP, no gap)
```

**Why the old method hopped** is in 4.4 (`STOP` sets volume 0 and the restart tick is silent on all channels). The self-loop never stops, so there is no seam to hear. Confirmed by ear on the real hardware.

**Details that matter**

* **Delay sum:** if one channel sums to `L+1` or `L-1` it drifts by one tick against the others every loop. The simulator catches it (see below).
* **Call depth:** an entry track other than 0 pushes one frame on its first `GOTO(itself)` (every channel starts with "current track = 0"). Entry track 0 (CH3 in Quest Theme) pushes none. Count this against the limit of 7.
* **Intro + body:** give every channel an *entry track* `E = [GOTO intro, GOTO loopT]` and a *loop track* `loopT = [GOTO body, GOTO loopT]`. The intro plays once; the loop track repeats the body forever and calls itself. The body must start by setting everything it depends on (`ATM_VOL`, `ATM_SL_VOL`, `ATM_SET_TRA`, `ATM_ARP`). Star Light (`extras/tools/gen_starlight.py`) is built this way. A leftover slide counter of the intro can change a volume by a few units for a few ticks in the first body pass only; the note events are identical.
* **Cost:** `GOTO` is 2 bytes where `STOP` was 1 (+1 per channel), `GOTO_ADV` (5 bytes) disappears, and every `_last` variant track (10 to 40 bytes plus 2 in the table) disappears because the last bar is the same as every other pass of that bar. Converting Italo Night saved 89 bytes (558 to 469), Escaper Droid 89, Neon Heart 89, Star Light 147.
* **Tempo command in the loop:** a song whose drum track (the usual CH3 entry, track 0) begins with `ATM_SET_TEMPO` re-executes it every pass. It sets the same value, so it is harmless.

**Proof, not belief.** Simulate three loops (`extras/tools/atmsim.py`, `extras/tools/gen_quest.py`). All of these must hold:

* `restarts == []` and no `STOP` after tick 0 (an unused channel's `STOP` at tick 0 is fine)
* for every tick `t >= L`: `TIMELINE[t] == TIMELINE[t-L]` for all four channels (volume, and frequency whenever the volume is not 0; a silent channel's last pitch is irrelevant)
* the note events of loop 1, 2 and 3 are identical
* max call depth <= 7

**Converting an old STOP + GOTO_ADV song:** `convert.py` (`extras/tools/convert.py`) does it automatically: it adds 1 to the last executed delay of every channel (the old loop was `L-1` ticks of delay plus one restart tick), turns the entry track's final `STOP` into `GOTO(entry)`, deletes `GOTO_ADV`, merges the tracks that became identical, and checks the result against the old song with the simulator and the real `ATMcmds.h`.

**One-shot songs:** no self-loop, no `GOTO_ADV`; channels end with `ATM_STOP_CHAN` and may end at different times; the song ends when the last one stops. For fades use `ATM_VOL` steps per section or `ATM_SL_VOL`.

---

## 7. Fitting a byte budget

Costs: note 1, delay 1, `VOL/SL_VOL/SET_TRA/ADD_TRA/SET_TEMPO` 2, `ARP` 3, `GOTO` 2, `REPEAT` 3, `RETURN/STOP` 1, plus 2 per track in the table and 5 for the header.

What saves bytes (in the order that mattered):

1. **Transposition instead of copies.** One 2-bar riff (30 B) plays over every chord with `SET_TRA`/`ADD_TRA` (2 B per use). The bass bar is written once in the key of A and transposed the same way.
2. **`REPEAT`** for identical bars: 3 bytes for any number of repeats.
3. **Shared tracks** pay off when a pattern of length `L >= 5` is used at least twice.
4. **Relative transposition cycles**: a track of `[riff][ADD_TRA(5) riff][ADD_TRA(-5) riff]` returns to its start key, so the whole cycle can be re-played in another key by one `SET_TRA`.
5. **Inline tiny things** (a 4-byte track costs 6 with its table entry).
6. **Drums**: one 24-tick core shared by every half-bar, called with `REPEAT(30, ...)`.
7. Chords as explicit long notes with an arp: `NOTE, DELAY(64), NOTE, DELAY(64)` is 4 bytes per two bars.

Quest Theme budget (233 bytes, self-loop): header 29 B, drums 39 (entry + core + half), lead 68, chords 41, bass 56. (The old STOP + `GOTO_ADV` version was 247 B: it needed the 5-byte `GOTO_ADV`, a trimmed `bass_bar_last` track and `STOP`s.)

### 7.1 Making a song smaller without changing it (Italo Night: 469 B to 365 B)

`italoDiscoCompressed` (`examples/loops/italoDiscoCompressed`) plays exactly like `italoNight` and is 104 bytes smaller (22 %). Counting from the first version: 558 B (`STOP` + `GOTO_ADV`), 469 B (self-loop), 365 B (compressed). The work is structure, not notes. In order of savings:

1. **Inline every track that is called once.** A track used once costs its table entry (2), its `RETURN` (1) and the call (2): 5 bytes for nothing. The 558 B version of Italo Night had 33 tracks (header 71 bytes); the compressed one has 16 (header 37 bytes).
2. **Use the entry track as the body.** With the self-loop (section 6) the lead is one track: all 8 bars in a row, then `ATM_GOTO(itself)`. The bass and chords are the same. No `_last` tracks, no trimmed last delays.
3. **Share a track only when it pays.** A track of length `L` used `u` times costs `L + 1 + 2 + 2u` against `u x L` inline, so it wins when `(u - 1) x L > 2u + 3`: two uses need `L >= 8`, three uses `L >= 5`.
4. **Repeat bigger blocks.** The drums play 4 bars, twice. Put those 4 bars in one track (`REPEAT(6, pair)`, beat A, roll, delay) and call it with `REPEAT(1, half)`: the second half costs nothing. The same applies to the chords (`REPEAT(3, beat_X)` per bar).
5. **Share the common head.** Every chord stab starts with the same kick (`ARP_OFF, SL_FRQ, VOL, NOTE, DELAY, VOL(0), DELAY, SL_VOL, VOL`). It is one `kick` track called by the five beat tracks. After the self-loop the 4th beat of the last bar is the same beat track as the others, so it needs no special case.
6. **Transpose instead of copy, and share the start of near-identical beats.** One bass bar track, eight `SET_TRA, GOTO bar` pairs (4 bytes per bar). The bass beats `silence, root, octave, root` and `silence, root, root, octave` both start with `VOL(0), DELAY(4), VOL(63), root, DELAY(4)`: that head is one 8-byte track, the two beats only add their last two notes (5 bytes saved). Where several pieces differ only at the end, share the head. Snare roll: the slide is set once, the roll is inlined in the drum half (it is only used there), and equal neighbours collapse into one `REPEAT` (two equal bars next to each other: `REPEAT(1, bar)`).
7. **Drop commands that do nothing.** Slides, volume and arpeggio are state that persists, so a second `SL_VOL(-3)` or `ARP_OFF` with the same value is dead weight. Only remove one after the simulator proves the timeline is unchanged.

**What did not pay.** Sharing the lead between bars: bars 1 to 4 have the same rhythm (12, 12, 8, 12, 12, 8) but different intervals (E C A, F C A, E C G, D B G), so a shared track would need a different transposition and one changed note per bar, and the extra track costs more than the 5 to 9 bytes it saves. Look for repetition in the *byte stream*, not in the rhythm: a candidate is only worth a track when the same sequence of commands (or the same sequence shifted by a constant) appears with `(u - 1) x L > 2u + 3`.

**How to be sure it is the same song** (the check used here, all four must hold):

* all note events of the new song equal those of the old song, tick for tick;
* the per-tick `TIMELINE` (volume and frequency of the 4 channels) equals the old one. When the old song is a `STOP` + `GOTO_ADV` loop, compare over one pass and expect differences only in the old silent restart tick. A silent channel's frequency may differ (inaudible);
* loops 1, 2 and 3 of the new song are identical, `restarts == []`, no `STOP` after tick 0, call depth <= 7;
* the compiled header equals the generator's bytes.

---

## 8. Composition recipes that worked

### 8.1 Drums (CH3 NOISE), copied from Italo Night

16-tick beats, all with `VOL` + `SL_VOL` + `DELAY`:

| Sound | Commands |
|---|---|
| kick click | `VOL(48) SL_VOL(-12) DELAY(4)` (Italo: `VOL(28) SL_VOL(-14)`) |
| closed hat | `VOL(14) SL_VOL(-7) DELAY(4)` (Italo: `VOL(12) SL_VOL(-6)`) |
| open hat | `VOL(26) SL_VOL(-3) DELAY(8)` |
| snare / clap | `VOL(44) SL_VOL(-5) DELAY(8)` (Italo: `VOL(40)`) |
| snare roll (8 steps of 2 ticks) | `VOL(22..50 in steps of 4) SL_VOL(-9) DELAY(2)` |

Beat A (beats 1, 3) = kick(4) + closed hat(4) + open hat(8). Beat B (beats 2, 4) = snare(8) + open hat(8). Fills: replace beat 4 of bar 4 and bar 8 with the roll.

### 8.2 Kick on a tonal channel (Italo Night CH1)

`ATM_ARP_OFF, ATM_SL_FRQ((uint8_t)-127), ATM_VOL(30), ATM_NOTE_D3_, ATM_DELAY(4), ATM_VOL(0), ATM_DELAY(4)`. Needs `ARP_OFF` first. Use when a channel is free of arpeggios.

### 8.3 Bass (CH2 SAW)

`VOL(63), SL_VOL(-5)`; one beat track, `bar = REPEAT(2, beat) + beat4`, with a variation on beat 4 (root, root, octave); the whole bar is written once in the key of A and transposed per chord.
* Italo Night beat (16 ticks): `VOL(0) DELAY(4)` (duck under the kick), `VOL(63)`, root(4), octave(4), root(4).
* Quest Theme beat (16 ticks): root(8), octave(8); beat 4: root(8), root(4), octave(4).

### 8.4 Lead (CH0 PULSE)

`VOL(38), SL_VOL(-2)`: every note starts loud and decays. Write a 1- or 2-bar riff and transpose it per chord.
**Which riffs survive transposition:** use the *pentatonic* notes of the chord.
* Minor key, chords i, iv, v (root offsets 0, +5, +7): minor pentatonic degrees 0, 3, 5, 7, 10 stay inside the key on all three chords. Avoid the 2nd degree on the v chord.
* Major key, chords I, IV, V: major pentatonic degrees 0, 2, 4, 7, 9.
* A riff containing the third only transposes cleanly between chords of the same quality. Otherwise leave the third out or write explicit notes.

### 8.5 Arpeggio chords (CH1 SQUARE)

* Stabs (Italo Night): `SL_VOL(-3), VOL(24), ARP(0x34, 0x20), NOTE, DELAY(8)` once per beat, after the kick.
* Held chords (Quest Theme): `VOL(30), SL_VOL(-1), ARP(0x34, 0x20)`, then `NOTE root, DELAY(64)` once per bar; the arpeggio shimmers and fades, then re-articulates on the bar.
* Root notes explicit (no transposition on this channel, see 4.6).

### 8.6 Form

* 8 bars (Italo Night, Escaper Droid): one pass of the chord progression, ~15 s at tempo 17/34.
* 16 bars (Quest Theme): the same 8-bar cycle twice, second time lifted a whole step with one `SET_TRA(2)`, ending on a pivot chord (Em: chord iv of Bm and chord v of Am) so the loop returns smoothly.
* Chord progressions used: Am | F | C | G | Am | F | G | E (Italo Night; bass transposition offsets from A: 0, -4, +3, -2, 0, -4, -2, -5), D | G | A | D | D | Bm | A | D (Escaper Droid; offsets from D: 0, +5, +7, 0, 0, +9, +7, 0), Am Am Dm Dm Am Am Em Em then +2 (Quest Theme).

### 8.7 Tiny in-game loops (50 to 100 bytes)

**The 30-song sketch.** All small in-game loops now live in ONE sketch, `examples/loops/inGameSongs` (`song.h` holds a `PROGMEM` pointer array, the A button switches song, the screen shows the name and its byte size, B pauses). Songs 1-5 are the five below, 6-10 chiptune, 11-15 Sega, 16-20 NES, 21-25 C64, 26-30 Game Boy style, each 50 to 100 bytes (list in section 13). Remember that Arduino keeps a stale `build` folder: if the IDE still shows 25 songs after an update, clean the build.

**Volume.** These loops are written at full volume. If the game also plays sound effects, scale them to about 60 % (see *Sound effects*, "Keep the music quieter than the effects").

The five songs in `ingame5.h` (Meadow Waltz 94 B, Lantern Lake 73 B, Sunday Market 89 B, Pixel Breeze 87 B, Music Box Tide 87 B) were written to show how little a pleasant, smooth, looping background track needs. What they share:

* **No drums unless the groove needs them** (Lantern Lake, Meadow Waltz, Pixel Breeze, Music Box Tide have none; Sunday Market uses one 16-tick brush track with `REPEAT`). Harmony and a good bass do the work.
* **Self-loop on every channel** (section 6): the entry track is the whole part and ends with `ATM_GOTO(itself)`. With only 3 or 4 tracks the header is 11 to 15 bytes.
* **Unused channels** point at a 1-byte `ATM_STOP_CHAN` track.
* **One long note per bar + an arpeggio** is the cheapest chord: `ATM_ARP(0x34, 0x23)` once, then `NOTE, DELAY(64)` per bar = 3 bytes per bar. Slow arps (`0x23`: 4 ticks per step) sound dreamy; fast ones (`0x20`) sound like stabs.
* **Soft decay** instead of drums for rhythm: `ATM_VOL(30), ATM_SL_VOL(-1)` makes every note a bell.
* **One riff, many chords:** Pixel Breeze plays one 6-note pentatonic riff over `C G F C` with one cycle track moved by `ATM_ADD_TRA`; the bass calls the same track from `ATM_SET_TRA(-24)`.
* **Rhythmic tricks that cost nothing:** 3-3-2 bossa grid (24+24+16 ticks); 5 against 3 phasing (a 5-note pattern of 8-tick notes against a 3-note arp at 8 ticks per step realigns every 120 ticks, exactly at the chord change, Music Box Tide).
* **Waltz** = 36 ticks per bar (3 beats of 12), loop 288 = 8 bars.
* **Tempo is the hardware value** here (20 to 32); `ATM_SET_TEMPO` costs 2 bytes, leave it out if the sketch calls `ATM.setTempo()`.

Complete example, Lantern Lake (73 bytes, D minor, tempo 20, loop 256 ticks = 12.8 s). Each channel is one self-looping track:

```cpp
Song lanternLake[] = {          // total song in bytes = 73
  0x04,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x2D, 0x00,                   // Address of track 2   //45
  0x3B, 0x00,                   // Address of track 3   //59

  0x00,                         // Channel 0 entry track (bells)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" bells  [24 bytes] - two bell strikes per bar, one per half bar
  ATM_SET_TEMPO(20),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_A5, ATM_DELAY(32),
  ATM_NOTE_F5, ATM_DELAY(32),
  ATM_NOTE_D6, ATM_DELAY(32),
  ATM_NOTE_A5_, ATM_DELAY(32),
  ATM_NOTE_A5, ATM_DELAY(32),
  ATM_NOTE_C6, ATM_DELAY(32),
  ATM_NOTE_G5, ATM_DELAY(32),
  ATM_NOTE_E5, ATM_DELAY(32),
  ATM_GOTO(0),            // loop: this very track

  //"Track 1" pad  [21 bytes] - slow arpeggio chords: Dm | Bb | F | C
  ATM_VOL(26),
  ATM_TREM(1, 7),
  ATM_ARP(0x34, 0x23),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_ARP(0x43, 0x23),
  ATM_NOTE_A3_, ATM_DELAY(64),
  ATM_NOTE_F4, ATM_DELAY(64),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_GOTO(1),

  //"Track 2" bass  [14 bytes] - one soft root per bar
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_D3, ATM_DELAY(64),
  ATM_NOTE_A2_, ATM_DELAY(64),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_GOTO(2),

  //"Track 3" silent  [1 byte] - unused channel stops at once
  ATM_STOP_CHAN,
};
```

All three musical channels sum to exactly 256 ticks (8 x 32, 4 x 64, 4 x 64). The pad's arpeggio is set before the first note and its two chord qualities are set before their notes (minor `0x34`, major `0x43`); the second pass starts with the arpeggio still on `0x43` from the last bar, but `ATM_ARP(0x34, 0x23)` at the top of the track resets it before the first note, so every pass is identical. A WAV preview of all five songs was made with a small offline synth (pulse 25 %, square, saw, noise; tick rate = tempo), not with ATMlib itself.

---

## 9. Tooling

All scripts live in `extras/tools/` (Python 3, plus `g++` for the byte check). Run them from that folder, with `ATM_SRC` pointing at `ATMlib/src`.

| File | What it is |
|---|---|
| `atmsim.py` | Python model of the playroutine's control flow **and** per-tick state: call stack, track-0 behaviour, delays, `REPEAT` counters, transposition, volume retrigger, arpeggio, volume/frequency slides (with the missing upper clamp), tremolo, vibrato, glissando, note-cut gate (it raises if cut is used with transposition), tempo commands, cues, and the STOP / `GOTO_ADV` restart rule for old files. It does **not** render audio. After `simulate()`, `atmsim.TIMELINE[tick]` holds `(volume, frequency_hz)` for each of the four channels, which is how fades, the cut gate and slides are checked. It models the long delay (`0xE0`). `atmsim.DELAYLOG` (tick, channel, address of every delay executed) and `atmsim.STOPLOG` (tick, channel, address of every `STOP`) support converters and loop checks. |
| `gen_quest.py` | Complete generator of Quest Theme (self-loop, 233 B): command constructors, named tracks, assembler (offsets, track numbers), verifier, header-file writer. The pieces to reuse for a small song. |
| `gen_starlight.py` | Generator of Star Light (intro + looping body, about 90 tracks, section-table check). The pattern for a long song. |
| `gen_orbital.py`, `render_orbital.py` | Generator and `.h` writer of Orbital Rush (3 minutes, 1479 B), same structure as Star Light. |
| `gen_cutlass.py` | Showcase song that uses nearly every effect: fades, cues, tempo changes, glissando, tremolo. |
| `convert.py` | Converts an old `STOP` + `GOTO_ADV` loop to the hop-free self-loop and verifies the result (section 6). |
| `check_header.cpp` | Compiles a song header against the real `ATMcmds.h` and compares the bytes with the generator's bytes. |

**Verification checklist (all must pass)**

- [ ] `sizeof(song)` within the byte budget
- [ ] **loops:** `restarts == []` and no `STOP` after tick 0 (unused channels excepted); no `ATM_GOTO_ADV`
- [ ] **loops:** every channel's entry track ends with `ATM_GOTO(itself)`, and every channel sums to exactly `L` ticks
- [ ] **loops:** `TIMELINE[t] == TIMELINE[t-L]` for all `t >= L` (volume, and frequency while audible)
- [ ] loops 1, 2, 3 have identical note events
- [ ] max call depth <= 7
- [ ] no track calls track 0
- [ ] every `note + transposition` is within 1..63
- [ ] compiled header bytes == generator bytes (`extras/tools/check_header.cpp`)
- [ ] tempo is the doubled value (in-game songs: the stated hardware value)
- [ ] one-shots only: every channel ends with `STOP`, no `GOTO_ADV`

---

## 10. Output file template

```cpp
#ifndef SONG_H
#define SONG_H

// ---------------------------------------------------------------------------
//  "Title" - style, key, bars, bytes
//  Play with:   #include "quest.h"      ATM.play(questTheme);
//  Timing : ATM_SET_TEMPO(34) (doubled). 1 beat = 16 ticks, 1 bar = 64 ticks ...
//  Form / channels. Loop: every entry track ends with ATM_GOTO(itself), no STOP, no hop.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif

Song questTheme[] = {     // total song bytes = 233
  0x0C,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0   0   drums
  ...                         // one line per track
  0x03,                       // CH0 entry -> track 3 (lead)
  0x06,                       // CH1 entry
  0x07,                       // CH2 entry
  0x00,                       // CH3 entry
  //"Track 0" drums  [10b]
  ATM_SET_TEMPO(34),
  ATM_REPEAT(30, 2),   // 31x drum_half
  ATM_GOTO(1),   // -> drum_core
  ATM_DELAY(8),
  ATM_GOTO(0),   // loop: this very track
  ...
};

#endif
```

`extras/tools/gen_quest.py` writes exactly this.

---

## 11. Using a song in a sketch

See *Quick start* and *API* at the top of this file. In short: `#include "song.h"`, create `ATMsynth ATM;`, call `ATM.play(<array name>)` in `setup()`.

**Folder layout (current):** `ATMlib/examples/loops/<name>/` for songs that loop forever (Italo Night, Italo Disco Compressed, Escaper Droid, Neon Heart, Quest Theme, Star Light, inGameSongs), `ATMlib/examples/songs/<name>/` for one-shots and long songs (Cutlass Cove, Midnight Run, Orbital Rush), `ATMlib/examples/soundEffects/` for effects (`sfxDemo`: 45 poke + 45 `playSfx` effects and 5 songs). Each song is an Arduino sketch folder with `<name>.ino`, `song.h`, `ATMconfig.h`, `bitmaps.h`: duplicate an existing folder, rename the folder and the `.ino` (the Arduino IDE wants both to match), put the data in `song.h` and change `ATM.play(<name>)`.

**Several songs in one sketch:** keep the song arrays in `PROGMEM` pointer arrays (see `examples/loops/inGameSongs`).

---

## 12. Mistakes made on this project (do not repeat)

1. **Invented opcodes.** A first Pixel Chase draft used guessed byte values (`0xC0` for volume ...). They collide with the delay range `0xA0..0xDF`. Always take values from `ATMcmds.h`.
2. **Track 0 called** (section 4.3). Looks fine in a tick counter, sounds like a stuck melody on hardware. A simulator that models the call stack catches it.
3. **`REPEAT(n)` is n+1 runs.** `REPEAT(7, beat)` made a 128-tick bar instead of 64.
4. **`GOTO_ADV` in a one-shot song** makes it restart. (Looping songs must not use it either, see mistake 11.)
5. **Off-by-one loops:** with the self-loop every channel must sum to exactly `L`; one channel off by a tick drifts against the others. (The old rule was `L-1`; do not mix the two.)
6. **Tempo not doubled.**
7. **Invalid names/arguments:** `ATM_NOTE_F#5`, `ATM_NOTE_OFF`, negative arguments without `(uint8_t)`.
8. **Plain, short, hex-looking songs** are not accepted. Use the Italo Night structure: real melody, kick/hat/snare with fills, transposed bass, chord stabs, a form with a destination.
9. **Combining `ATM_ARP` with transposition on one channel** (detunes the arpeggio).
10. **Trusting a counter instead of a model.** Verify with the simulator and by compiling.
11. **Looping with `STOP` + `GOTO_ADV`.** Every loop has one tick in which all four channels are silent (`STOP` sets volume 0, the restart tick plays nothing). The period was exact and the simulator's "all stops on tick `L-1`" check passed, yet a "hop" was audible in every song. The model has to check *what is audible* (the per-tick timeline), not only when events happen. Fixed by the self-loop (section 6).
12. **Believing "gapless on paper".** The first attempt at a gapless version (trimming one tick and keeping `STOP`) was wrong for the same reason; only removing `STOP` from the loop removes the seam.

---

## 13. Catalog of the existing songs

| Folder (in `examples/`) | Array | Style | Size | Notes |
|---|---|---|---|---|
| `loops/ItaloDisco` | `italoNight` | Italo disco, A minor, 8 bars | 469 B | Written at emulator tempo 17 (now `SET_TEMPO(35)` in the file; use the doubled value). Kick + arp stabs on CH1, transposed SAW bass, noise drums with fills. Chords Am F C G Am F G E. **Self-loop** (was 558 B with STOP + `GOTO_ADV`). |
| `loops/italoDiscoCompressed` | `italoDiscoCompressed` | Italo disco, A minor, 8 bars | 365 B | Same song as `italoNight`, rewritten to be smaller (section 7.1): 16 tracks instead of 33, self-loop, `SET_TEMPO(35)`. Tick-for-tick identical playback. Generator pattern: `extras/tools/gen_quest.py`. |
| `loops/escaperDroid` | `escaperDroid` | uptempo space, D major, 8 bars | 455 B | Same skeleton as Italo Night. ARP `0x47` / `0x37`. `ATM_CUE(1)` once per beat (inside the kick track, 2 bytes) for the game's dancing droid sprite. Self-loop (was 544 B). |
| `loops/neonHeart` | `neonHeart` | Italo disco, E minor, 8 bars | 476 B | Tempo 36. Italo Night skeleton with new key, melody and chords: Em C G D Em C D B (dominant pulls back to Em). Chord stabs use inversions (Em, C/E, G/D, D/F#, B/D#). Self-loop (was 565 B). |
| `songs/midnightRun` | `midnightRun` | 80s synthwave, A minor, 96 bars (~3 min) | 715 B | One-shot, fade in and out, no `GOTO_ADV`. |
| `loops/questTheme` | `questTheme` | in-game chiptune, A minor, 16 bars | 233 B | Tempo 34 inside the song (remove `ATM_SET_TEMPO` if the sketch sets it). Arpeggio chords, transposed lead and bass, key lift and Em pivot, **self-loop** (was 247 B). Best reference for tight budgets. Generator: `extras/tools/gen_quest.py`. |
| `loops/inGameSongs` | `meadowWaltz` | folk waltz in 3/4, C major, 8 bars of 36 ticks | 94 B | Tempo 26, loop 288 ticks. Melody written out, oom-pah-pah bass. Self-loop. See 8.7. |
| `loops/inGameSongs` | `lanternLake` | ambient / dreamy, D minor, 4 bars | 73 B | Tempo 20, loop 256 ticks. Bells, slow arpeggio pad with tremolo, long bass roots. Listed in full in 8.7. |
| `loops/inGameSongs` | `sundayMarket` | bossa-nova lounge, A dorian, 4 bars | 89 B | Tempo 32, loop 256 ticks. Minor-7 arp stabs (`0x37`), 3-3-2 bass, brush noise with `REPEAT`. |
| `loops/inGameSongs` | `pixelBreeze` | light chiptune, C major pentatonic, 4 bars | 87 B | Tempo 30, loop 256 ticks. One riff moved by `ADD_TRA` over C G F C, bass calls the same cycle track. |
| `loops/inGameSongs` | `musicBoxTide` | minimal music box, phasing 5 against 3, C G F | 87 B | Tempo 30, loop 360 ticks. 5-note pattern against a 3-note arp, realigns at each chord change. |
| *(not in this tree)* | `gameOver` | one-shot game over jingle, A minor, 5 bars | 106 B | Tempo 20. Andalusian descent Am-G-F-E-Am, one lead phrase transposed down with ADD_TRA, arp chords, bass on the same transposition, noise toll per bar. No GOTO_ADV, so it ends by itself. Reference for small one-shots (a game's own `youDied` does the same in 64 B). |
| `songs/cutlassCove` | `cutlassCove` | original tropical-pirate showcase, C major / A minor, 32 bars (~75 s) | 839 B | One-shot, tempo 30 (+6 in B, ritardando in the outro), fades, cues 1..6. Uses nearly every command; generator `extras/tools/gen_cutlass.py`. An original piece, not a copy of any existing song. |
| `loops/starLight` | `starLight` | original K-pop style dance track, C major, 3:00, intro + 84-bar body that loops forever | 1307 B | Tempo 33 (~124 BPM). Verse / pre-chorus / chorus / dance break / bridge / key-lifted last chorus (+2). Hook, pre motif and dance riff are one track each, transposed per chord. **Self-loop with intro** (was 1454 B). Generator: `extras/tools/gen_starlight.py`. |
| `songs/orbitalRush` | `orbitalRush` | original demoscene style (Purple Motion / Jonne Valtonen flavour, nothing copied), D minor, 3:00 first pass, intro + 96-bar body that loops forever | 1479 B | Tempo 37 (~139 BPM). A / B / vibrato break with glissando riser / chorus / dance riff / chorus lifted +2 (Em C Am B) / tag back into A. Bass is one 16th-note bar transposed per chord. **Self-loop with intro.** Generator pattern = `extras/tools/gen_starlight.py`. |
| `loops/inGameSongs` | `coinRush` | Chiptune style: bouncy arcade chiptune, C major | 88 B | Tempo 36, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `bitBounce` | Chiptune style: syncopated chiptune, G major | 94 B | Tempo 34, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `starlitRun` | Chiptune style: driving minor chiptune, A minor | 91 B | Tempo 38, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `caveCrawl` | Chiptune style: slow dark chiptune, D minor | 76 B | Tempo 24, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `victoryLap` | Chiptune style: march-like fanfare chiptune, F major | 81 B | Tempo 40, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `turboStreet` | Sega style: funky Mega Drive slap bass, E minor | 93 B | Tempo 36, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `sunsetStrip` | Sega style: smooth jazz-pop with vibrato lead, D major | 95 B | Tempo 30, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `chromeRider` | Sega style: driving night-ride rock, B minor | 85 B | Tempo 40, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `marbleRuins` | Sega style: mysterious arpeggios, C minor | 95 B | Tempo 28, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `velvetLounge` | Sega style: swung late-night lounge, F major | 99 B | Tempo 32, loop 384 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `castleGate` | NES style: dramatic minor with pumping bass, E minor | 98 B | Tempo 38, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `dungeonDash` | NES style: fast minor run, A minor | 90 B | Tempo 44, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `overworldMorning` | NES style: bright marching adventure, C major | 91 B | Tempo 40, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `fortressRun` | NES style: tense staccato chase, D minor | 86 B | Tempo 42, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `pixelParade` | NES style: cheerful 3/4 stroll, G major | 97 B | Tempo 28, loop 192 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `breadbinBeat` | C64 style: fast arpeggio chords + gallop bass, A minor | 88 B | Tempo 38, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `sidSunrise` | C64 style: bright rolling arpeggios, D major | 98 B | Tempo 36, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `loaderLullaby` | C64 style: slow 3/4 lullaby, G minor | 90 B | Tempo 22, loop 288 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `rasterRain` | C64 style: fast 16th lead + steady kick, E minor | 91 B | Tempo 40, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `joystickJam` | C64 style: funky minor groove, C minor | 92 B | Tempo 36, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `pocketQuest` | Game Boy style: bright handheld adventure, C major | 84 B | Tempo 38, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `handheldHero` | Game Boy style: perky marching pulses, G major | 91 B | Tempo 42, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `linkCable` | Game Boy style: playful 3/4 bounce, A minor | 99 B | Tempo 30, loop 192 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `brickLogic` | Game Boy style: thoughtful staccato puzzle loop, D minor | 86 B | Tempo 34, loop 256 ticks. Self-loop, one track per channel. |
| `loops/inGameSongs` | `dmgDawn` | Game Boy style: sleepy menu music, F major | 78 B | Tempo 24, loop 256 ticks. Self-loop, one track per channel. |
| `soundEffects/sfxDemo` | `pipeDream` | NES style: pulse arpeggio hops over an octave-bouncing bass, C major (C Am F G) | 104 B | Tempo 42, loop 256 ticks, 60 % volume (`MV` / `MS`). Self-loop. |
| `soundEffects/sfxDemo` | `canyonRaid` | Atari 2600 style: wavering low square lead, saw pump bass, noise thump, A minor (Am G F E) | 102 B | Tempo 36, loop 256 ticks, 60 % volume. Lead on CH1, CH0 silent. |
| `soundEffects/sfxDemo` | `greenZone` | Sega style: bright pulse lead, syncopated bass, hat-heavy beat, C major (C Bb F G) | 106 B | Tempo 44, loop 256 ticks, 60 % volume. |
| `soundEffects/sfxDemo` | `neonHighway` | Sega style: arpeggio lead, answering melody, driving 16th bass, D minor (Dm Bb F C) | 108 B | Tempo 44, loop 256 ticks, 60 % volume. All four channels used. |
| *(removed)* | `pixelChase` | NES-like | 181 B | **Broken (track-0 bug), rejected. Do not use.** |

---

---

## Credits and license

* Davey Taylor (STG): Squawk, ATMsynth, effects
* Joeri Gantois (JO3RI): original ATMlib song format, effects, TEAM a.r.g. tools
* This tree: command macros with byte counts, compile-time waveforms / effects / functions / octaves, `playSfx`, cues, the self-loop method, the example songs, the simulator and generators, the HTML tracker and this documentation

See also [t-arg.github.io](https://t-arg.github.io/) and the library notes on the Arduino Library Manager listing for ATMlib. Licensed under the BSD 3-Clause License, see `LICENSE`.
