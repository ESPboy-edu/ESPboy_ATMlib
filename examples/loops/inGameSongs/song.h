#ifndef INGAME_SONGS_H
#define INGAME_SONGS_H

// ---------------------------------------------------------------------------
//  30 small in-game loops for ATMlib (50 - 100 bytes each), written with ATM_* commands.
//  1-5 mixed styles, 6-10 chiptune, 11-15 Sega style, 16-20 NES style, 21-25 C64 style, 26-30 Game Boy style.
//
//  Play with:   #include "song.h"      ATM.play(meadowWaltz);   (any of the names below)
//
//  All of them loop forever with NO restart gap. Each channel ends its main track with ATM_GOTO to that
//  same track. A call to the track you are already in does not push the stack: it simply jumps back
//  to the first command in the same tick, so no tick is lost and no volume is reset.
//  (ATM_STOP_CHAN + ATM_GOTO_ADV, the usual loop, silences every channel for one tick at the seam.)
//  Every channel must add up to exactly the loop length in ticks, or it drifts against the others.
//  The tempo is the real hardware value (ticks per second); it is set inside each song.
//  Unused channels point at a track that holds a single ATM_STOP_CHAN (1 byte).
//  Track 0 is never called from another track (it is only a channel entry).
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif

// ===========================================================================
//  "Meadow Waltz" - folk waltz in 3/4, C major
//  94 bytes | tempo 26 (ticks per second) | loop 288 ticks = 11.1 s
//  Form: 8 bars of 36 ticks | C  Am  F  G | C  Am  F  G | (G leads back into C)
//    CH0 PULSE : the melody, written out once, 8 bars, no repeats (2 notes per bar keeps it cheap)
//    CH2 SAW   : oom-pah-pah (root, fifth, third) for C | Am | F | G. One 4-bar track, called twice
//    CH1/CH3   : unused, they point at a one-byte STOP track
//    Loop      : every channel ends with ATM_GOTO to its own track (see the header comment)
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song meadowWaltz[] = {          // total song in bytes = 94
  0x04,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x2C, 0x00,                   // Address of track 1   //44
  0x38, 0x00,                   // Address of track 2   //56
  0x39, 0x00,                   // Address of track 3   //57

  0x00,                         // Channel 0 entry track (lead)
  0x02,                         // Channel 1 entry track (silent)
  0x01,                         // Channel 2 entry track (bass)
  0x02,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [44 bytes] - melody, 8 bars of 36 ticks, written out (no repeats)
  ATM_SET_TEMPO(26),
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-2),
  ATM_NOTE_G5, ATM_DELAY(24),
  ATM_NOTE_E5, ATM_DELAY(12),
  ATM_NOTE_C6, ATM_DELAY(18),
  ATM_NOTE_B5, ATM_DELAY(6),
  ATM_NOTE_A5, ATM_DELAY(12),
  ATM_NOTE_A5, ATM_DELAY(24),
  ATM_NOTE_F5, ATM_DELAY(12),
  ATM_NOTE_D6, ATM_DELAY(24),
  ATM_NOTE_B5, ATM_DELAY(12),
  ATM_NOTE_C6, ATM_DELAY(24),
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_NOTE_A5, ATM_DELAY(18),
  ATM_NOTE_G5, ATM_DELAY(6),
  ATM_NOTE_E5, ATM_DELAY(12),
  ATM_NOTE_F5, ATM_DELAY(24),
  ATM_NOTE_A5, ATM_DELAY(12),
  ATM_NOTE_D6, ATM_DELAY(24),
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_GOTO(0),            // -> lead

  //"Track 1" bass  [12 bytes] - oom-pah-pah, the 4-bar cycle twice
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_GOTO(3),            // -> bassA
  ATM_DELAY(12),
  ATM_GOTO(3),            // -> bassA
  ATM_DELAY(12),
  ATM_GOTO(1),            // -> bass

  //"Track 2" silent  [1 bytes] - unused channels stop at once
  ATM_STOP_CHAN,

  //"Track 3" bassA  [24 bytes] - C | Am | F | G as root, fifth, third; the last delay is added by the caller
  ATM_NOTE_C3, ATM_DELAY(12),
  ATM_NOTE_G3, ATM_DELAY(12),
  ATM_NOTE_E3, ATM_DELAY(12),
  ATM_NOTE_A2, ATM_DELAY(12),
  ATM_NOTE_E3, ATM_DELAY(12),
  ATM_NOTE_C3, ATM_DELAY(12),
  ATM_NOTE_F2, ATM_DELAY(12),
  ATM_NOTE_C3, ATM_DELAY(12),
  ATM_NOTE_A2, ATM_DELAY(12),
  ATM_NOTE_G2, ATM_DELAY(12),
  ATM_NOTE_D3, ATM_DELAY(12),
  ATM_NOTE_B2,
  ATM_RETURN,

};

// ===========================================================================
//  "Lantern Lake" - ambient / dreamy, D minor
//  73 bytes | tempo 20 (ticks per second) | loop 256 ticks = 12.8 s
//  Form: 4 bars of 64 ticks | Dm  Bb  F  C
//    CH0 PULSE : two soft bell strikes per bar (fifth and third of each chord), decaying with ATM_SL_VOL
//    CH1 SQUARE: one slow arpeggio chord per bar (4 ticks per step) with a gentle tremolo
//    CH2 SAW   : one long soft root per bar
//    No drums, no transposition: the harmony does all the work, so the song is only 73 bytes
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

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
  ATM_GOTO(0),            // -> bells

  //"Track 1" pad  [21 bytes] - slow arpeggio chords (4 ticks per step) with a gentle tremolo: Dm | Bb | F | C
  ATM_VOL(26),
  ATM_TREM(1, 7),
  ATM_ARP(0x34, 0x23),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_ARP(0x43, 0x23),
  ATM_NOTE_A3_, ATM_DELAY(64),
  ATM_NOTE_F4, ATM_DELAY(64),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [14 bytes] - one soft root per bar
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_D3, ATM_DELAY(64),
  ATM_NOTE_A2_, ATM_DELAY(64),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Sunday Market" - bossa-nova lounge, A dorian
//  89 bytes | tempo 32 (ticks per second) | loop 256 ticks = 8.0 s
//  Form: 4 bars of 64 ticks | Am7  Dm7  Am7  Em7
//    CH1 SQUARE: minor-7th chord stabs (ATM_ARP 0x37, 1 tick per step) on ticks 0 and 24 of every bar; bar 4 adds a third hit
//    CH2 SAW   : bossa bass, root-fifth-root on the 3-3-2 grid (24+24+16 ticks)
//    CH3 NOISE : loud-soft brush on every eighth note; one 16-tick track repeated with ATM_REPEAT
//    CH0       : unused
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song sundayMarket[] = {          // total song in bytes = 89
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1D, 0x00,                   // Address of track 1   //29
  0x3B, 0x00,                   // Address of track 2   //59
  0x42, 0x00,                   // Address of track 3   //66
  0x49, 0x00,                   // Address of track 4   //73

  0x04,                         // Channel 0 entry track (silent)
  0x00,                         // Channel 1 entry track (comp)
  0x01,                         // Channel 2 entry track (bass)
  0x02,                         // Channel 3 entry track (shaker)

  //"Track 0" comp  [29 bytes] - Am7 | Dm7 | Am7 | Em7, two stabs per bar (3 + 5 eighths)
  ATM_SET_TEMPO(32),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-2),
  ATM_ARP(0x37, 0x20),
  ATM_NOTE_A3, ATM_DELAY(24),
  ATM_NOTE_A3, ATM_DELAY(40),
  ATM_NOTE_D4, ATM_DELAY(24),
  ATM_NOTE_D4, ATM_DELAY(40),
  ATM_NOTE_A3, ATM_DELAY(24),
  ATM_NOTE_A3, ATM_DELAY(40),
  ATM_NOTE_E4, ATM_DELAY(24),
  ATM_NOTE_E4, ATM_DELAY(24),
  ATM_NOTE_E4, ATM_DELAY(16),
  ATM_GOTO(0),            // -> comp

  //"Track 1" bass  [30 bytes] - root-fifth-root on the 3-3-2 grid
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_A2, ATM_DELAY(24),
  ATM_NOTE_E3, ATM_DELAY(24),
  ATM_NOTE_A2, ATM_DELAY(16),
  ATM_NOTE_D3, ATM_DELAY(24),
  ATM_NOTE_A3, ATM_DELAY(24),
  ATM_NOTE_D3, ATM_DELAY(16),
  ATM_NOTE_A2, ATM_DELAY(24),
  ATM_NOTE_E3, ATM_DELAY(24),
  ATM_NOTE_A2, ATM_DELAY(16),
  ATM_NOTE_E3, ATM_DELAY(24),
  ATM_NOTE_B3, ATM_DELAY(24),
  ATM_NOTE_E3, ATM_DELAY(16),
  ATM_GOTO(1),            // -> bass

  //"Track 2" shaker  [7 bytes] - brush: loud-soft eighth notes all through the loop
  ATM_SL_VOL((uint8_t)-3),
  ATM_REPEAT(15, 3),       // 16x tick
  ATM_GOTO(2),            // -> shaker

  //"Track 3" tick  [7 bytes] - loud tick + soft tick = one beat
  ATM_VOL(14),
  ATM_DELAY(8),
  ATM_VOL(6),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Pixel Breeze" - light chiptune, C major pentatonic
//  87 bytes | tempo 30 (ticks per second) | loop 256 ticks = 8.5 s
//  Form: 4 bars of 64 ticks | C  G  F  C (the last bar an octave up)
//    ONE riff track (pentatonic, 6 notes) plays over all four chords: a shared "cycle" track moves it with ATM_ADD_TRA
//    CH0 PULSE and CH2 SAW both call the same cycle track; the bass just starts 24 semitones lower (ATM_SET_TRA(-24))
//    CH1 SQUARE: slow major arpeggio chords, explicit roots (arpeggio and transposition must not share a channel)
//    The octave jump of the last bar costs one ATM_ADD_TRA
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song pixelBreeze[] = {          // total song in bytes = 87
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x0D, 0x00,                   // Address of track 1   //13
  0x1C, 0x00,                   // Address of track 2   //28
  0x27, 0x00,                   // Address of track 3   //39
  0x28, 0x00,                   // Address of track 4   //40
  0x3A, 0x00,                   // Address of track 5   //58

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (chords)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [13 bytes] - the riff on top (pulse)
  ATM_SET_TEMPO(30),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(4),            // -> cycle
  ATM_DELAY(8),
  ATM_GOTO(0),            // -> lead

  //"Track 1" chords  [15 bytes] - slow major arpeggio chords: C | G | F | C
  ATM_VOL(16),
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_GOTO(1),            // -> chords

  //"Track 2" bass  [11 bytes] - the same riff two octaves down (saw): same cycle, different start transposition
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA((uint8_t)-24),
  ATM_GOTO(4),            // -> cycle
  ATM_DELAY(8),
  ATM_GOTO(2),            // -> bass

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

  //"Track 4" cycle  [18 bytes] - C G F C(+12); the last delay of the final bar is added by the caller
  ATM_GOTO(5),            // -> riff
  ATM_DELAY(8),
  ATM_ADD_TRA(7),
  ATM_GOTO(5),            // -> riff
  ATM_DELAY(8),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(5),            // -> riff
  ATM_DELAY(8),
  ATM_ADD_TRA(7),
  ATM_GOTO(5),            // -> riff
  ATM_RETURN,

  //"Track 5" riff  [12 bytes] - pentatonic riff, 64 ticks minus the last delay
  ATM_NOTE_C4, ATM_DELAY(16),
  ATM_NOTE_D4, ATM_DELAY(8),
  ATM_NOTE_G4, ATM_DELAY(16),
  ATM_NOTE_A4, ATM_DELAY(8),
  ATM_NOTE_G4, ATM_DELAY(8),
  ATM_NOTE_D4,
  ATM_RETURN,

};

// ===========================================================================
//  "Music Box Tide" - minimal music box (phasing)
//  87 bytes | tempo 30 (ticks per second) | loop 360 ticks = 12.0 s
//  Form: 3 segments of 120 ticks | C  G  F  (360 ticks)
//    CH0 PULSE : a 5-note pattern (8 ticks each = 40 ticks), 3 runs per segment
//    CH1 SQUARE: ATM_ARP triad at 8 ticks per step = a 3-note pattern (24 ticks), 5 turns per segment
//    5 against 3: the two patterns drift against each other and realign every 120 ticks, exactly at the chord change
//    CH2 SAW   : one soft root per segment. The last run of the music box drops its last two notes (a small cadence)
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song musicBoxTide[] = {          // total song in bytes = 87
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1D, 0x00,                   // Address of track 1   //29
  0x2D, 0x00,                   // Address of track 2   //45
  0x3C, 0x00,                   // Address of track 3   //60
  0x3D, 0x00,                   // Address of track 4   //61

  0x00,                         // Channel 0 entry track (box)
  0x01,                         // Channel 1 entry track (arp)
  0x02,                         // Channel 2 entry track (drone)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" box  [29 bytes] - five-note box (40 ticks) = 3 runs per 120-tick segment
  ATM_SET_TEMPO(30),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(2, 4),       // 3x five
  ATM_ADD_TRA((uint8_t)-5),
  ATM_REPEAT(2, 4),       // 3x five
  ATM_ADD_TRA((uint8_t)-2),
  ATM_REPEAT(1, 4),       // 2x five
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_A5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(24),
  ATM_ADD_TRA(7),
  ATM_GOTO(0),            // -> box

  //"Track 1" arp  [16 bytes] - major triad arpeggio per segment: C | G | F
  ATM_VOL(22),
  ATM_ARP(0x43, 0x27),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_DELAY(56),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_DELAY(56),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_DELAY(56),
  ATM_GOTO(1),            // -> arp

  //"Track 2" drone  [15 bytes] - soft root per segment
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_DELAY(56),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_DELAY(56),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_DELAY(56),
  ATM_GOTO(2),            // -> drone

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

  //"Track 4" five  [11 bytes] - G A E C D
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_A5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_C6, ATM_DELAY(8),
  ATM_NOTE_D6, ATM_DELAY(8),
  ATM_RETURN,

};

// ===========================================================================
//  "Coin Rush" - bouncy arcade chiptune, C major   [Chiptune style]
//  88 bytes | tempo 36 (ticks per second) | loop 256 ticks = 7.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song coinRush[] = {          // total song in bytes = 88
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x29, 0x00,                   // Address of track 2   //41
  0x43, 0x00,                   // Address of track 3   //67
  0x48, 0x00,                   // Address of track 4   //72

  0x00,                         // Channel 0 entry track (lead)
  0x04,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [24 bytes] - C | Am | F | G, one riff moved by ADD_TRA
  ATM_SET_TEMPO(36),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [17 bytes]
  ATM_NOTE_C5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_C6, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" bass  [26 bytes] - octave bounce
  ATM_VOL(50),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_REPEAT(3, 3),       // 4x oct
  ATM_ADD_TRA((uint8_t)-3),
  ATM_REPEAT(3, 3),       // 4x oct
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(3, 3),       // 4x oct
  ATM_ADD_TRA(2),
  ATM_REPEAT(3, 3),       // 4x oct
  ATM_GOTO(2),            // -> bass

  //"Track 3" oct  [5 bytes]
  ATM_NOTE_C3, ATM_DELAY(8),
  ATM_NOTE_C4, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Bit Bounce" - syncopated chiptune, G major   [Chiptune style]
//  94 bytes | tempo 34 (ticks per second) | loop 256 ticks = 7.5 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song bitBounce[] = {          // total song in bytes = 94
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x29, 0x00,                   // Address of track 2   //41
  0x40, 0x00,                   // Address of track 3   //64
  0x4E, 0x00,                   // Address of track 4   //78

  0x00,                         // Channel 0 entry track (lead)
  0x02,                         // Channel 1 entry track (pad)
  0x03,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [24 bytes] - G | Em | C | D
  ATM_SET_TEMPO(34),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(8),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [17 bytes]
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_NOTE_B5, ATM_DELAY(4),
  ATM_NOTE_D6, ATM_DELAY(8),
  ATM_NOTE_B5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_NOTE_A5, ATM_DELAY(4),
  ATM_NOTE_B5, ATM_DELAY(8),
  ATM_NOTE_D6, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" pad  [23 bytes] - held arpeggio chords
  ATM_VOL(18),
  ATM_SL_VOL((uint8_t)-1),
  ATM_ARP(0x43, 0x21),
  ATM_NOTE_G4, ATM_DELAY(64),
  ATM_ARP(0x34, 0x21),
  ATM_NOTE_E4, ATM_DELAY(64),
  ATM_ARP(0x43, 0x21),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_GOTO(2),            // -> pad

  //"Track 3" bass  [14 bytes]
  ATM_VOL(52),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_NOTE_E2, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_NOTE_D3, ATM_DELAY(64),
  ATM_GOTO(3),            // -> bass

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Starlit Run" - driving minor chiptune, A minor   [Chiptune style]
//  91 bytes | tempo 38 (ticks per second) | loop 256 ticks = 6.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song starlitRun[] = {          // total song in bytes = 91
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1C, 0x00,                   // Address of track 1   //28
  0x2D, 0x00,                   // Address of track 2   //45
  0x3B, 0x00,                   // Address of track 3   //59
  0x42, 0x00,                   // Address of track 4   //66
  0x49, 0x00,                   // Address of track 5   //73

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [28 bytes] - Am | F | C | G
  ATM_SET_TEMPO(38),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_REPEAT(1, 1),       // 2x arp
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(1, 1),       // 2x arp
  ATM_ADD_TRA(7),
  ATM_REPEAT(1, 1),       // 2x arp
  ATM_ADD_TRA((uint8_t)-5),
  ATM_REPEAT(1, 1),       // 2x arp
  ATM_GOTO(0),            // -> lead

  //"Track 1" arp  [17 bytes]
  ATM_NOTE_A4, ATM_DELAY(4),
  ATM_NOTE_C5, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_A5, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_C5, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_C5, ATM_DELAY(4),
  ATM_RETURN,

  //"Track 2" bass  [14 bytes]
  ATM_VOL(56),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_A2, ATM_DELAY(64),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [7 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(42),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Cave Crawl" - slow dark chiptune, D minor   [Chiptune style]
//  76 bytes | tempo 24 (ticks per second) | loop 256 ticks = 10.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song caveCrawl[] = {          // total song in bytes = 76
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1F, 0x00,                   // Address of track 1   //31
  0x2F, 0x00,                   // Address of track 2   //47
  0x38, 0x00,                   // Address of track 3   //56
  0x3C, 0x00,                   // Address of track 4   //60

  0x00,                         // Channel 0 entry track (lead)
  0x04,                         // Channel 1 entry track (silent)
  0x01,                         // Channel 2 entry track (bass)
  0x02,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [31 bytes]
  ATM_SET_TEMPO(24),
  ATM_VOL(32),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(2, 3),
  ATM_NOTE_D5, ATM_DELAY(32),
  ATM_NOTE_F5, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_NOTE_D5, ATM_DELAY(24),
  ATM_NOTE_A4, ATM_DELAY(40),
  ATM_NOTE_G5, ATM_DELAY(32),
  ATM_NOTE_F5, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_NOTE_F5, ATM_DELAY(24),
  ATM_NOTE_E5, ATM_DELAY(40),
  ATM_GOTO(0),            // -> lead

  //"Track 1" bass  [16 bytes]
  ATM_VOL(48),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_D2, ATM_DELAY(64),
  ATM_NOTE_D2, ATM_DELAY(64),
  ATM_NOTE_A2, ATM_DELAY(64),
  ATM_NOTE_A2, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_GOTO(1),            // -> bass

  //"Track 2" drums  [9 bytes]
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(7, 3),       // 8x drip
  ATM_GOTO(2),            // -> drums

  //"Track 3" drip  [4 bytes]
  ATM_VOL(34),
  ATM_DELAY(32),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Victory Lap" - march-like fanfare chiptune, F major   [Chiptune style]
//  81 bytes | tempo 40 (ticks per second) | loop 256 ticks = 6.4 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song victoryLap[] = {          // total song in bytes = 81
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x23, 0x00,                   // Address of track 2   //35
  0x31, 0x00,                   // Address of track 3   //49
  0x38, 0x00,                   // Address of track 4   //56
  0x3F, 0x00,                   // Address of track 5   //63

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [24 bytes] - F | Bb | C | F
  ATM_SET_TEMPO(40),
  ATM_VOL(38),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [11 bytes]
  ATM_NOTE_C5, ATM_DELAY(12),
  ATM_NOTE_C5, ATM_DELAY(4),
  ATM_NOTE_C5, ATM_DELAY(8),
  ATM_NOTE_F5, ATM_DELAY(16),
  ATM_NOTE_A5, ATM_DELAY(24),
  ATM_RETURN,

  //"Track 2" bass  [14 bytes]
  ATM_VOL(52),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_NOTE_A2_, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [7 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(42),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Turbo Street" - funky Mega Drive slap bass, E minor   [Sega style]
//  93 bytes | tempo 36 (ticks per second) | loop 256 ticks = 7.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song turboStreet[] = {          // total song in bytes = 93
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x19, 0x00,                   // Address of track 1   //25
  0x24, 0x00,                   // Address of track 2   //36
  0x38, 0x00,                   // Address of track 3   //56
  0x4D, 0x00,                   // Address of track 4   //77

  0x00,                         // Channel 0 entry track (lead)
  0x04,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [25 bytes] - Em Em C D
  ATM_SET_TEMPO(36),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(2, 3),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [11 bytes]
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_A5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" bass  [20 bytes] - slap bass: short decaying notes
  ATM_VOL(60),
  ATM_SL_VOL((uint8_t)-6),
  ATM_SET_TRA(0),
  ATM_GOTO(3),            // -> slap
  ATM_GOTO(3),            // -> slap
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(3),            // -> slap
  ATM_ADD_TRA(2),
  ATM_GOTO(3),            // -> slap
  ATM_GOTO(2),            // -> bass

  //"Track 3" slap  [21 bytes]
  ATM_NOTE_E2, ATM_DELAY(8),
  ATM_NOTE_E2, ATM_DELAY(4),
  ATM_NOTE_E3, ATM_DELAY(4),
  ATM_NOTE_E2, ATM_DELAY(8),
  ATM_NOTE_G2, ATM_DELAY(8),
  ATM_NOTE_E2, ATM_DELAY(4),
  ATM_NOTE_A2, ATM_DELAY(4),
  ATM_NOTE_B2, ATM_DELAY(8),
  ATM_NOTE_E3, ATM_DELAY(8),
  ATM_NOTE_D3, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Sunset Strip" - smooth jazz-pop with vibrato lead, D major   [Sega style]
//  95 bytes | tempo 30 (ticks per second) | loop 256 ticks = 8.5 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song sunsetStrip[] = {          // total song in bytes = 95
  0x04,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x23, 0x00,                   // Address of track 1   //35
  0x3B, 0x00,                   // Address of track 2   //59
  0x51, 0x00,                   // Address of track 3   //81

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [35 bytes]
  ATM_SET_TEMPO(30),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(3, 3),
  ATM_NOTE_F5_, ATM_DELAY(24),
  ATM_NOTE_A5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_F5_, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(32),
  ATM_NOTE_D5, ATM_DELAY(32),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_F5_, ATM_DELAY(32),
  ATM_NOTE_A5, ATM_DELAY(16),
  ATM_NOTE_B5, ATM_DELAY(16),
  ATM_NOTE_A5, ATM_DELAY(32),
  ATM_GOTO(0),            // -> lead

  //"Track 1" pad  [24 bytes] - Dmaj7 | Em7 | Gmaj7 | Am7
  ATM_VOL(20),
  ATM_ARP(0x47, 0x22),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_ARP(0x37, 0x22),
  ATM_NOTE_E4, ATM_DELAY(64),
  ATM_ARP(0x47, 0x22),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_ARP(0x37, 0x22),
  ATM_NOTE_A3, ATM_DELAY(64),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [22 bytes]
  ATM_VOL(50),
  ATM_SL_VOL((uint8_t)-2),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_NOTE_A2, ATM_DELAY(32),
  ATM_NOTE_E3, ATM_DELAY(32),
  ATM_NOTE_B2, ATM_DELAY(32),
  ATM_NOTE_G2, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_NOTE_A2, ATM_DELAY(32),
  ATM_NOTE_E3, ATM_DELAY(32),
  ATM_GOTO(2),            // -> bass

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Chrome Rider" - driving night-ride rock, B minor   [Sega style]
//  85 bytes | tempo 40 (ticks per second) | loop 256 ticks = 6.4 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song chromeRider[] = {          // total song in bytes = 85
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1B, 0x00,                   // Address of track 1   //27
  0x24, 0x00,                   // Address of track 2   //36
  0x32, 0x00,                   // Address of track 3   //50
  0x3C, 0x00,                   // Address of track 4   //60
  0x43, 0x00,                   // Address of track 5   //67

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [27 bytes] - Bm | A | D | G as offsets
  ATM_SET_TEMPO(40),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(2, 3),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-8),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [9 bytes]
  ATM_NOTE_B4, ATM_DELAY(24),
  ATM_NOTE_D5, ATM_DELAY(8),
  ATM_NOTE_F5_, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" bass  [14 bytes]
  ATM_VOL(58),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_B2, ATM_DELAY(64),
  ATM_NOTE_A2, ATM_DELAY(64),
  ATM_NOTE_D3, ATM_DELAY(64),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [10 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(14),
  ATM_DELAY(4),
  ATM_VOL(42),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Marble Ruins" - mysterious arpeggios, C minor   [Sega style]
//  95 bytes | tempo 28 (ticks per second) | loop 256 ticks = 9.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song marbleRuins[] = {          // total song in bytes = 95
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1B, 0x00,                   // Address of track 1   //27
  0x32, 0x00,                   // Address of track 2   //50
  0x40, 0x00,                   // Address of track 3   //64
  0x49, 0x00,                   // Address of track 4   //73
  0x4D, 0x00,                   // Address of track 5   //77

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [27 bytes]
  ATM_SET_TEMPO(28),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_TREM(1, 7),
  ATM_NOTE_G5, ATM_DELAY(48),
  ATM_NOTE_D5_, ATM_DELAY(16),
  ATM_NOTE_F5, ATM_DELAY(32),
  ATM_NOTE_D5, ATM_DELAY(32),
  ATM_NOTE_C5, ATM_DELAY(32),
  ATM_NOTE_D5_, ATM_DELAY(32),
  ATM_NOTE_G5, ATM_DELAY(32),
  ATM_NOTE_F5, ATM_DELAY(32),
  ATM_GOTO(0),            // -> lead

  //"Track 1" pad  [23 bytes]
  ATM_VOL(22),
  ATM_SL_VOL((uint8_t)-1),
  ATM_ARP(0x34, 0x22),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_G3_, ATM_DELAY(64),
  ATM_ARP(0x34, 0x22),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [14 bytes]
  ATM_VOL(46),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_C2, ATM_DELAY(64),
  ATM_NOTE_C2, ATM_DELAY(64),
  ATM_NOTE_G2_, ATM_DELAY(64),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" drums  [9 bytes]
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-4),
  ATM_REPEAT(3, 4),       // 4x toll
  ATM_GOTO(3),            // -> drums

  //"Track 4" toll  [4 bytes]
  ATM_VOL(26),
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Velvet Lounge" - swung late-night lounge, F major   [Sega style]
//  99 bytes | tempo 32 (ticks per second) | loop 384 ticks = 12.0 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song velvetLounge[] = {          // total song in bytes = 99
  0x04,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x23, 0x00,                   // Address of track 1   //35
  0x3F, 0x00,                   // Address of track 2   //63
  0x55, 0x00,                   // Address of track 3   //85

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [35 bytes]
  ATM_SET_TEMPO(32),
  ATM_VOL(32),
  ATM_NOTE_A5, ATM_DELAY(16),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_F5, ATM_DELAY(24),
  ATM_NOTE_A5, ATM_DELAY(48),
  ATM_NOTE_A5_, ATM_DELAY(16),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(64),
  ATM_DELAY(8),
  ATM_NOTE_A5, ATM_DELAY(16),
  ATM_NOTE_F5, ATM_DELAY(8),
  ATM_NOTE_D5, ATM_DELAY(24),
  ATM_NOTE_F5, ATM_DELAY(48),
  ATM_NOTE_G5, ATM_DELAY(24),
  ATM_NOTE_E5, ATM_DELAY(24),
  ATM_NOTE_C5, ATM_DELAY(48),
  ATM_GOTO(0),            // -> lead

  //"Track 1" pad  [28 bytes]
  ATM_VOL(20),
  ATM_ARP(0x47, 0x22),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_DELAY(32),
  ATM_ARP(0x47, 0x22),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_DELAY(32),
  ATM_ARP(0x37, 0x22),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_DELAY(32),
  ATM_ARP(0x46, 0x22),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_DELAY(32),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [22 bytes]
  ATM_VOL(48),
  ATM_SL_VOL((uint8_t)-2),
  ATM_NOTE_F2, ATM_DELAY(48),
  ATM_NOTE_C3, ATM_DELAY(48),
  ATM_NOTE_C3, ATM_DELAY(48),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_NOTE_D3, ATM_DELAY(48),
  ATM_NOTE_A3, ATM_DELAY(48),
  ATM_NOTE_G2, ATM_DELAY(48),
  ATM_NOTE_D3, ATM_DELAY(48),
  ATM_GOTO(2),            // -> bass

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Castle Gate" - dramatic minor with pumping bass, E minor   [NES style]
//  98 bytes | tempo 38 (ticks per second) | loop 256 ticks = 6.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song castleGate[] = {          // total song in bytes = 98
  0x07,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x23, 0x00,                   // Address of track 2   //35
  0x3D, 0x00,                   // Address of track 3   //61
  0x40, 0x00,                   // Address of track 4   //64
  0x47, 0x00,                   // Address of track 5   //71
  0x4E, 0x00,                   // Address of track 6   //78

  0x00,                         // Channel 0 entry track (lead)
  0x06,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x05,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [24 bytes] - Em | C | D | B as offsets
  ATM_SET_TEMPO(38),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [11 bytes]
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_F5_, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_NOTE_B4, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" bass  [26 bytes]
  ATM_VOL(58),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_REPEAT(7, 3),       // 8x eight
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(7, 3),       // 8x eight
  ATM_ADD_TRA(2),
  ATM_REPEAT(7, 3),       // 8x eight
  ATM_ADD_TRA((uint8_t)-3),
  ATM_REPEAT(7, 3),       // 8x eight
  ATM_GOTO(2),            // -> bass

  //"Track 3" eight  [3 bytes]
  ATM_NOTE_E2, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" beat  [7 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(42),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 5" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 4),       // 16x beat
  ATM_GOTO(5),            // -> drums

  //"Track 6" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Dungeon Dash" - fast minor run, A minor   [NES style]
//  90 bytes | tempo 44 (ticks per second) | loop 256 ticks = 5.8 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song dungeonDash[] = {          // total song in bytes = 90
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1A, 0x00,                   // Address of track 1   //26
  0x2B, 0x00,                   // Address of track 2   //43
  0x43, 0x00,                   // Address of track 3   //67
  0x4A, 0x00,                   // Address of track 4   //74

  0x00,                         // Channel 0 entry track (lead)
  0x04,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [26 bytes] - Am Am F G
  ATM_SET_TEMPO(44),
  ATM_VOL(32),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_REPEAT(1, 1),       // 2x run
  ATM_REPEAT(1, 1),       // 2x run
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(1, 1),       // 2x run
  ATM_ADD_TRA(2),
  ATM_REPEAT(1, 1),       // 2x run
  ATM_GOTO(0),            // -> lead

  //"Track 1" run  [17 bytes]
  ATM_NOTE_A4, ATM_DELAY(4),
  ATM_NOTE_C5, ATM_DELAY(4),
  ATM_NOTE_A4, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_A4, ATM_DELAY(4),
  ATM_NOTE_G5, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_C5, ATM_DELAY(4),
  ATM_RETURN,

  //"Track 2" bass  [24 bytes]
  ATM_VOL(56),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_ADD_TRA(2),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_GOTO(2),            // -> bass

  //"Track 3" gal  [7 bytes]
  ATM_NOTE_A2, ATM_DELAY(8),
  ATM_NOTE_A2, ATM_DELAY(4),
  ATM_NOTE_A3, ATM_DELAY(4),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Overworld Morning" - bright marching adventure, C major   [NES style]
//  91 bytes | tempo 40 (ticks per second) | loop 256 ticks = 6.4 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song overworldMorning[] = {          // total song in bytes = 91
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x25, 0x00,                   // Address of track 2   //37
  0x3B, 0x00,                   // Address of track 3   //59
  0x42, 0x00,                   // Address of track 4   //66
  0x49, 0x00,                   // Address of track 5   //73

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [24 bytes] - C | F | G | C
  ATM_SET_TEMPO(40),
  ATM_VOL(38),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [13 bytes]
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_D5, ATM_DELAY(8),
  ATM_NOTE_C5, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" bass  [22 bytes]
  ATM_VOL(52),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_C3, ATM_DELAY(32),
  ATM_NOTE_G3, ATM_DELAY(32),
  ATM_NOTE_F3, ATM_DELAY(32),
  ATM_NOTE_C4, ATM_DELAY(32),
  ATM_NOTE_G3, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_NOTE_C3, ATM_DELAY(32),
  ATM_NOTE_G3, ATM_DELAY(32),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [7 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(42),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Fortress Run" - tense staccato chase, D minor   [NES style]
//  86 bytes | tempo 42 (ticks per second) | loop 256 ticks = 6.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song fortressRun[] = {          // total song in bytes = 86
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x29, 0x00,                   // Address of track 2   //41
  0x43, 0x00,                   // Address of track 3   //67
  0x46, 0x00,                   // Address of track 4   //70

  0x00,                         // Channel 0 entry track (lead)
  0x04,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [24 bytes] - Dm | C | F | Dm
  ATM_SET_TEMPO(42),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> stac
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(1),            // -> stac
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> stac
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(1),            // -> stac
  ATM_GOTO(0),            // -> lead

  //"Track 1" stac  [17 bytes]
  ATM_NOTE_D5, ATM_DELAY(8),
  ATM_NOTE_D5, ATM_DELAY(8),
  ATM_NOTE_F5, ATM_DELAY(8),
  ATM_NOTE_D5, ATM_DELAY(8),
  ATM_NOTE_A5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_F5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" bass  [26 bytes]
  ATM_VOL(58),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_REPEAT(7, 3),       // 8x p
  ATM_ADD_TRA((uint8_t)-2),
  ATM_REPEAT(7, 3),       // 8x p
  ATM_ADD_TRA(5),
  ATM_REPEAT(7, 3),       // 8x p
  ATM_ADD_TRA((uint8_t)-3),
  ATM_REPEAT(7, 3),       // 8x p
  ATM_GOTO(2),            // -> bass

  //"Track 3" p  [3 bytes]
  ATM_NOTE_D2, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Pixel Parade" - cheerful 3/4 stroll, G major   [NES style]
//  97 bytes | tempo 28 (ticks per second) | loop 192 ticks = 6.9 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song pixelParade[] = {          // total song in bytes = 97
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x21, 0x00,                   // Address of track 2   //33
  0x32, 0x00,                   // Address of track 3   //50
  0x48, 0x00,                   // Address of track 4   //72
  0x4F, 0x00,                   // Address of track 5   //79

  0x00,                         // Channel 0 entry track (lead)
  0x02,                         // Channel 1 entry track (pad)
  0x03,                         // Channel 2 entry track (bass)
  0x05,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [24 bytes] - G | C | D | G, 4 bars of 48
  ATM_SET_TEMPO(28),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [9 bytes]
  ATM_NOTE_D5, ATM_DELAY(12),
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_NOTE_B5, ATM_DELAY(12),
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_RETURN,

  //"Track 2" pad  [17 bytes]
  ATM_VOL(16),
  ATM_SL_VOL((uint8_t)-1),
  ATM_ARP(0x43, 0x21),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_NOTE_C4, ATM_DELAY(48),
  ATM_NOTE_D4, ATM_DELAY(48),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_GOTO(2),            // -> pad

  //"Track 3" bass  [22 bytes]
  ATM_VOL(50),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_GOTO(4),            // -> pah
  ATM_ADD_TRA(5),
  ATM_GOTO(4),            // -> pah
  ATM_ADD_TRA(2),
  ATM_GOTO(4),            // -> pah
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(4),            // -> pah
  ATM_GOTO(3),            // -> bass

  //"Track 4" pah  [7 bytes]
  ATM_NOTE_G2, ATM_DELAY(16),
  ATM_NOTE_D3, ATM_DELAY(16),
  ATM_NOTE_B2, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Breadbin Beat" - fast arpeggio chords + gallop bass, A minor   [C64 style]
//  88 bytes | tempo 38 (ticks per second) | loop 256 ticks = 6.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song breadbinBeat[] = {          // total song in bytes = 88
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x13, 0x00,                   // Address of track 1   //19
  0x27, 0x00,                   // Address of track 2   //39
  0x41, 0x00,                   // Address of track 3   //65
  0x48, 0x00,                   // Address of track 4   //72

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [19 bytes]
  ATM_SET_TEMPO(38),
  ATM_VOL(32),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(2, 3),
  ATM_NOTE_A5, ATM_DELAY(64),
  ATM_NOTE_G5, ATM_DELAY(64),
  ATM_NOTE_E5, ATM_DELAY(64),
  ATM_NOTE_D5, ATM_DELAY(64),
  ATM_GOTO(0),            // -> lead

  //"Track 1" pad  [20 bytes] - one-tick arpeggio = the SID chord sound
  ATM_VOL(20),
  ATM_SL_VOL(0),
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_A3, ATM_DELAY(64),
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [26 bytes]
  ATM_VOL(58),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_ADD_TRA(7),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_ADD_TRA((uint8_t)-5),
  ATM_REPEAT(3, 3),       // 4x gal
  ATM_GOTO(2),            // -> bass

  //"Track 3" gal  [7 bytes]
  ATM_NOTE_A2, ATM_DELAY(8),
  ATM_NOTE_A2, ATM_DELAY(4),
  ATM_NOTE_A3, ATM_DELAY(4),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "SID Sunrise" - bright rolling arpeggios, D major   [C64 style]
//  98 bytes | tempo 36 (ticks per second) | loop 256 ticks = 7.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song sidSunrise[] = {          // total song in bytes = 98
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1C, 0x00,                   // Address of track 1   //28
  0x2D, 0x00,                   // Address of track 2   //45
  0x3E, 0x00,                   // Address of track 3   //62
  0x52, 0x00,                   // Address of track 4   //82

  0x00,                         // Channel 0 entry track (lead)
  0x02,                         // Channel 1 entry track (pad)
  0x03,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [28 bytes] - D | G | A | D
  ATM_SET_TEMPO(36),
  ATM_VOL(32),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_REPEAT(1, 1),       // 2x roll
  ATM_ADD_TRA(5),
  ATM_REPEAT(1, 1),       // 2x roll
  ATM_ADD_TRA(2),
  ATM_REPEAT(1, 1),       // 2x roll
  ATM_ADD_TRA((uint8_t)-7),
  ATM_REPEAT(1, 1),       // 2x roll
  ATM_GOTO(0),            // -> lead

  //"Track 1" roll  [17 bytes]
  ATM_NOTE_D5, ATM_DELAY(4),
  ATM_NOTE_F5_, ATM_DELAY(4),
  ATM_NOTE_A5, ATM_DELAY(4),
  ATM_NOTE_F5_, ATM_DELAY(4),
  ATM_NOTE_D5, ATM_DELAY(4),
  ATM_NOTE_F5_, ATM_DELAY(4),
  ATM_NOTE_A5, ATM_DELAY(4),
  ATM_NOTE_D6, ATM_DELAY(4),
  ATM_RETURN,

  //"Track 2" pad  [17 bytes]
  ATM_VOL(18),
  ATM_SL_VOL(0),
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_NOTE_A3, ATM_DELAY(64),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_GOTO(2),            // -> pad

  //"Track 3" bass  [20 bytes]
  ATM_VOL(56),
  ATM_NOTE_D2, ATM_DELAY(32),
  ATM_NOTE_A2, ATM_DELAY(32),
  ATM_NOTE_G2, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_NOTE_A2, ATM_DELAY(32),
  ATM_NOTE_E3, ATM_DELAY(32),
  ATM_NOTE_D2, ATM_DELAY(32),
  ATM_NOTE_A2, ATM_DELAY(32),
  ATM_GOTO(3),            // -> bass

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Loader Lullaby" - slow 3/4 lullaby, G minor   [C64 style]
//  90 bytes | tempo 22 (ticks per second) | loop 288 ticks = 13.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song loaderLullaby[] = {          // total song in bytes = 90
  0x04,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1F, 0x00,                   // Address of track 1   //31
  0x3A, 0x00,                   // Address of track 2   //58
  0x4C, 0x00,                   // Address of track 3   //76

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [31 bytes]
  ATM_SET_TEMPO(22),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(2, 3),
  ATM_NOTE_D5, ATM_DELAY(24),
  ATM_NOTE_G5, ATM_DELAY(24),
  ATM_NOTE_F5, ATM_DELAY(48),
  ATM_NOTE_D5_, ATM_DELAY(24),
  ATM_NOTE_D5, ATM_DELAY(24),
  ATM_NOTE_C5, ATM_DELAY(48),
  ATM_NOTE_D5, ATM_DELAY(24),
  ATM_NOTE_G5, ATM_DELAY(24),
  ATM_NOTE_A5_, ATM_DELAY(24),
  ATM_NOTE_A5, ATM_DELAY(24),
  ATM_GOTO(0),            // -> lead

  //"Track 1" pad  [27 bytes]
  ATM_VOL(18),
  ATM_SL_VOL(0),
  ATM_ARP(0x34, 0x23),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_NOTE_C4, ATM_DELAY(48),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_ARP(0x43, 0x23),
  ATM_NOTE_A3_, ATM_DELAY(48),
  ATM_NOTE_D4, ATM_DELAY(48),
  ATM_ARP(0x34, 0x23),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [18 bytes]
  ATM_VOL(46),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_G2, ATM_DELAY(48),
  ATM_NOTE_C3, ATM_DELAY(48),
  ATM_NOTE_G2, ATM_DELAY(48),
  ATM_NOTE_A2_, ATM_DELAY(48),
  ATM_NOTE_D3, ATM_DELAY(48),
  ATM_NOTE_G2, ATM_DELAY(48),
  ATM_GOTO(2),            // -> bass

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Raster Rain" - fast 16th lead + steady kick, E minor   [C64 style]
//  91 bytes | tempo 40 (ticks per second) | loop 256 ticks = 6.4 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song rasterRain[] = {          // total song in bytes = 91
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1C, 0x00,                   // Address of track 1   //28
  0x2D, 0x00,                   // Address of track 2   //45
  0x3B, 0x00,                   // Address of track 3   //59
  0x42, 0x00,                   // Address of track 4   //66
  0x49, 0x00,                   // Address of track 5   //73

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [28 bytes] - Em | C | D | B
  ATM_SET_TEMPO(40),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_REPEAT(1, 1),       // 2x fast
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(1, 1),       // 2x fast
  ATM_ADD_TRA(2),
  ATM_REPEAT(1, 1),       // 2x fast
  ATM_ADD_TRA((uint8_t)-3),
  ATM_REPEAT(1, 1),       // 2x fast
  ATM_GOTO(0),            // -> lead

  //"Track 1" fast  [17 bytes]
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_G5, ATM_DELAY(4),
  ATM_NOTE_B5, ATM_DELAY(4),
  ATM_NOTE_G5, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_B4, ATM_DELAY(4),
  ATM_NOTE_E5, ATM_DELAY(4),
  ATM_NOTE_G5, ATM_DELAY(4),
  ATM_RETURN,

  //"Track 2" bass  [14 bytes]
  ATM_VOL(56),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_E2, ATM_DELAY(64),
  ATM_NOTE_C2, ATM_DELAY(64),
  ATM_NOTE_D2, ATM_DELAY(64),
  ATM_NOTE_B2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [7 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(14),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Joystick Jam" - funky minor groove, C minor   [C64 style]
//  92 bytes | tempo 36 (ticks per second) | loop 256 ticks = 7.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song joystickJam[] = {          // total song in bytes = 92
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x16, 0x00,                   // Address of track 1   //22
  0x23, 0x00,                   // Address of track 2   //35
  0x37, 0x00,                   // Address of track 3   //55
  0x4C, 0x00,                   // Address of track 4   //76

  0x00,                         // Channel 0 entry track (lead)
  0x04,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [22 bytes] - Cm Cm Fm G
  ATM_SET_TEMPO(36),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-2),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [13 bytes]
  ATM_NOTE_G5, ATM_DELAY(12),
  ATM_NOTE_G5, ATM_DELAY(4),
  ATM_NOTE_A5_, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_F5, ATM_DELAY(16),
  ATM_NOTE_D5_, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" bass  [20 bytes]
  ATM_VOL(60),
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(3),            // -> funk
  ATM_GOTO(3),            // -> funk
  ATM_ADD_TRA(5),
  ATM_GOTO(3),            // -> funk
  ATM_ADD_TRA(2),
  ATM_GOTO(3),            // -> funk
  ATM_GOTO(2),            // -> bass

  //"Track 3" funk  [21 bytes]
  ATM_NOTE_C2, ATM_DELAY(8),
  ATM_NOTE_C2, ATM_DELAY(4),
  ATM_NOTE_C3, ATM_DELAY(4),
  ATM_NOTE_C2, ATM_DELAY(8),
  ATM_NOTE_D2_, ATM_DELAY(8),
  ATM_NOTE_C2, ATM_DELAY(4),
  ATM_NOTE_F2, ATM_DELAY(4),
  ATM_NOTE_G2, ATM_DELAY(8),
  ATM_NOTE_C3, ATM_DELAY(8),
  ATM_NOTE_A2_, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Pocket Quest" - bright handheld adventure, C major   [Game Boy style]
//  84 bytes | tempo 38 (ticks per second) | loop 256 ticks = 6.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song pocketQuest[] = {          // total song in bytes = 84
  0x05,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x25, 0x00,                   // Address of track 2   //37
  0x36, 0x00,                   // Address of track 3   //54
  0x44, 0x00,                   // Address of track 4   //68

  0x00,                         // Channel 0 entry track (lead)
  0x02,                         // Channel 1 entry track (pad)
  0x03,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [24 bytes] - C | F | G | C
  ATM_SET_TEMPO(38),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [13 bytes]
  ATM_NOTE_C5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_C6, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_E5, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" pad  [17 bytes] - second pulse: quick chord arps
  ATM_VOL(14),
  ATM_SL_VOL(0),
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_GOTO(2),            // -> pad

  //"Track 3" bass  [14 bytes]
  ATM_VOL(46),
  ATM_SL_VOL(0),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_NOTE_G3, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_GOTO(3),            // -> bass

  //"Track 4" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Handheld Hero" - perky marching pulses, G major   [Game Boy style]
//  91 bytes | tempo 42 (ticks per second) | loop 256 ticks = 6.1 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song handheldHero[] = {          // total song in bytes = 91
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x25, 0x00,                   // Address of track 2   //37
  0x3B, 0x00,                   // Address of track 3   //59
  0x42, 0x00,                   // Address of track 4   //66
  0x49, 0x00,                   // Address of track 5   //73

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [24 bytes] - G | C | D | G
  ATM_SET_TEMPO(42),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-4),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [13 bytes]
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(8),
  ATM_NOTE_B5, ATM_DELAY(8),
  ATM_NOTE_D6, ATM_DELAY(8),
  ATM_NOTE_B5, ATM_DELAY(16),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 2" bass  [22 bytes]
  ATM_VOL(48),
  ATM_SL_VOL(0),
  ATM_NOTE_G2, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_NOTE_C3, ATM_DELAY(32),
  ATM_NOTE_G3, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_NOTE_A3, ATM_DELAY(32),
  ATM_NOTE_G2, ATM_DELAY(32),
  ATM_NOTE_D3, ATM_DELAY(32),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [7 bytes]
  ATM_VOL(48),
  ATM_DELAY(8),
  ATM_VOL(14),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Link Cable" - playful 3/4 bounce, A minor   [Game Boy style]
//  99 bytes | tempo 30 (ticks per second) | loop 192 ticks = 6.4 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song linkCable[] = {          // total song in bytes = 99
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x21, 0x00,                   // Address of track 2   //33
  0x36, 0x00,                   // Address of track 3   //54
  0x4A, 0x00,                   // Address of track 4   //74
  0x51, 0x00,                   // Address of track 5   //81

  0x00,                         // Channel 0 entry track (lead)
  0x02,                         // Channel 1 entry track (pad)
  0x03,                         // Channel 2 entry track (bass)
  0x05,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [24 bytes] - Am | F | G | Am, 4 bars of 48
  ATM_SET_TEMPO(30),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(2),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [9 bytes]
  ATM_NOTE_A5, ATM_DELAY(12),
  ATM_NOTE_E5, ATM_DELAY(12),
  ATM_NOTE_C6, ATM_DELAY(12),
  ATM_NOTE_E5, ATM_DELAY(12),
  ATM_RETURN,

  //"Track 2" pad  [21 bytes]
  ATM_VOL(14),
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_A3, ATM_DELAY(48),
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_F3, ATM_DELAY(48),
  ATM_NOTE_G3, ATM_DELAY(48),
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_A3, ATM_DELAY(48),
  ATM_GOTO(2),            // -> pad

  //"Track 3" bass  [20 bytes]
  ATM_VOL(46),
  ATM_SET_TRA(0),
  ATM_GOTO(4),            // -> pah
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(4),            // -> pah
  ATM_ADD_TRA(2),
  ATM_GOTO(4),            // -> pah
  ATM_ADD_TRA(2),
  ATM_GOTO(4),            // -> pah
  ATM_GOTO(3),            // -> bass

  //"Track 4" pah  [7 bytes]
  ATM_NOTE_A2, ATM_DELAY(16),
  ATM_NOTE_E3, ATM_DELAY(16),
  ATM_NOTE_E3, ATM_DELAY(16),
  ATM_RETURN,

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "Brick Logic" - thoughtful staccato puzzle loop, D minor   [Game Boy style]
//  86 bytes | tempo 34 (ticks per second) | loop 256 ticks = 7.5 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song brickLogic[] = {          // total song in bytes = 86
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x25, 0x00,                   // Address of track 2   //37
  0x33, 0x00,                   // Address of track 3   //51
  0x3D, 0x00,                   // Address of track 4   //61
  0x44, 0x00,                   // Address of track 5   //68

  0x00,                         // Channel 0 entry track (lead)
  0x05,                         // Channel 1 entry track (silent)
  0x02,                         // Channel 2 entry track (bass)
  0x04,                         // Channel 3 entry track (drums)

  //"Track 0" lead  [24 bytes] - Dm | C | F | Dm
  ATM_SET_TEMPO(34),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA(5),
  ATM_GOTO(1),            // -> riff
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(1),            // -> riff
  ATM_GOTO(0),            // -> lead

  //"Track 1" riff  [13 bytes]
  ATM_NOTE_D5, ATM_DELAY(16),
  ATM_NOTE_F5, ATM_DELAY(8),
  ATM_NOTE_A5, ATM_DELAY(8),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_F5, ATM_DELAY(8),
  ATM_NOTE_E5, ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" bass  [14 bytes]
  ATM_VOL(44),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_D2, ATM_DELAY(64),
  ATM_NOTE_C2, ATM_DELAY(64),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_NOTE_D2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [10 bytes]
  ATM_VOL(14),
  ATM_DELAY(8),
  ATM_VOL(14),
  ATM_DELAY(4),
  ATM_VOL(42),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL((uint8_t)-6),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

// ===========================================================================
//  "DMG Dawn" - sleepy menu music, F major   [Game Boy style]
//  78 bytes | tempo 24 (ticks per second) | loop 256 ticks = 10.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================

Song dmgDawn[] = {          // total song in bytes = 78
  0x04,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x1B, 0x00,                   // Address of track 1   //27
  0x32, 0x00,                   // Address of track 2   //50
  0x40, 0x00,                   // Address of track 3   //64

  0x00,                         // Channel 0 entry track (lead)
  0x01,                         // Channel 1 entry track (pad)
  0x02,                         // Channel 2 entry track (bass)
  0x03,                         // Channel 3 entry track (silent)

  //"Track 0" lead  [27 bytes]
  ATM_SET_TEMPO(24),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(2, 3),
  ATM_NOTE_A5, ATM_DELAY(48),
  ATM_NOTE_G5, ATM_DELAY(16),
  ATM_NOTE_F5, ATM_DELAY(32),
  ATM_NOTE_C5, ATM_DELAY(32),
  ATM_NOTE_D5, ATM_DELAY(32),
  ATM_NOTE_F5, ATM_DELAY(32),
  ATM_NOTE_E5, ATM_DELAY(32),
  ATM_NOTE_C5, ATM_DELAY(32),
  ATM_GOTO(0),            // -> lead

  //"Track 1" pad  [23 bytes]
  ATM_VOL(14),
  ATM_SL_VOL(0),
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_F3, ATM_DELAY(64),
  ATM_NOTE_A2_, ATM_DELAY(64),
  ATM_ARP(0x34, 0x22),
  ATM_NOTE_D4, ATM_DELAY(64),
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_C4, ATM_DELAY(64),
  ATM_GOTO(1),            // -> pad

  //"Track 2" bass  [14 bytes]
  ATM_VOL(44),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_NOTE_A2_, ATM_DELAY(64),
  ATM_NOTE_D3, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

#endif
