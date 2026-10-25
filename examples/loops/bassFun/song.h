#ifndef SONG_H
#define SONG_H

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif

// Play with:   #include "song.h"      ATM.play(bassFun);
// 1 beat = 16 ticks, 1 bar = 64 ticks, 8 bars = 512 ticks. Every channel is one self-looping track (no hop).
// Pitch: ATMlib note names sound about an octave above their label.

// ---------------------------------------------------------------------------
// Bass Fun - Italo disco groove (after the demo "Bass Fun")
// D A C G (half a bar each) four times = 8 bars (512 ticks, about 18 s), tempo 28 (105 BPM), last G ends on an F.
// CH0 pad (slow arpeggio chords), CH1 kick, CH2 bass: low note, octave jumps, ducked by the kick; notes taken from the MP3 (3 octaves up, ATM cannot go lower), CH3 noise clap + hats + a roll.
// 306 bytes
// ---------------------------------------------------------------------------
Song bassFun[] = {          // total song in bytes = 306
  0x13,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0  drums
  0x07, 0x00,                   // Address of track 1   //7  drum4
  0x0F, 0x00,                   // Address of track 2   //15  pair
  0x14, 0x00,                   // Address of track 3   //20  hat
  0x1D, 0x00,                   // Address of track 4   //29  clap
  0x28, 0x00,                   // Address of track 5   //40  roll
  0x43, 0x00,                   // Address of track 6   //67  kicks
  0x48, 0x00,                   // Address of track 7   //72  kick
  0x53, 0x00,                   // Address of track 8   //83  pad
  0x58, 0x00,                   // Address of track 9   //88  pad4
  0x70, 0x00,                   // Address of track 10   //112  bD1
  0x7C, 0x00,                   // Address of track 11   //124  bD2
  0x8B, 0x00,                   // Address of track 12   //139  bA
  0x98, 0x00,                   // Address of track 13   //152  bC1
  0xA9, 0x00,                   // Address of track 14   //169  bC2
  0xB6, 0x00,                   // Address of track 15   //182  bG1
  0xC2, 0x00,                   // Address of track 16   //194  bG2
  0xCD, 0x00,                   // Address of track 17   //205  bass
  0xD4, 0x00,                   // Address of track 18   //212  bass8

  0x08,                         // Channel 0 entry track (pad)
  0x06,                         // Channel 1 entry track (kicks)
  0x11,                         // Channel 2 entry track (bass)
  0x00,                         // Channel 3 entry track (drums)

  //"Track 0" drums  [7 bytes]
  ATM_SET_TEMPO(28),
  ATM_REPEAT(1, 1),   // 2x drum4
  ATM_GOTO(0),   // -> drums

  //"Track 1" drum4  [8 bytes]
  ATM_REPEAT(6, 2),   // 7x pair
  ATM_GOTO(3),   // -> hat
  ATM_GOTO(5),   // -> roll
  ATM_RETURN,

  //"Track 2" pair  [5 bytes]
  ATM_GOTO(3),   // -> hat
  ATM_GOTO(4),   // -> clap
  ATM_RETURN,

  //"Track 3" hat  [9 bytes]
  ATM_VOL(0),
  ATM_DELAY(8),
  ATM_VOL(14),
  ATM_SL_VOL((uint8_t)-4),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" clap  [11 bytes]
  ATM_VOL(44),
  ATM_SL_VOL((uint8_t)-6),
  ATM_DELAY(8),
  ATM_VOL(14),
  ATM_SL_VOL((uint8_t)-4),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 5" roll  [27 bytes]
  ATM_VOL(22),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(26),
  ATM_DELAY(2),
  ATM_VOL(30),
  ATM_DELAY(2),
  ATM_VOL(34),
  ATM_DELAY(2),
  ATM_VOL(38),
  ATM_DELAY(2),
  ATM_VOL(42),
  ATM_DELAY(2),
  ATM_VOL(46),
  ATM_DELAY(2),
  ATM_VOL(50),
  ATM_DELAY(2),
  ATM_RETURN,

  //"Track 6" kicks  [5 bytes]
  ATM_REPEAT(31, 7),   // 32x kick
  ATM_GOTO(6),   // -> kicks

  //"Track 7" kick  [11 bytes]
  ATM_ARP_OFF,
  ATM_SL_FRQ((uint8_t)-127),
  ATM_VOL(40),
  ATM_NOTE_D3_,
  ATM_DELAY(4),
  ATM_VOL(0),
  ATM_DELAY(12),
  ATM_RETURN,

  //"Track 8" pad  [5 bytes]
  ATM_REPEAT(1, 9),   // 2x pad4
  ATM_GOTO(8),   // -> pad

  //"Track 9" pad4  [24 bytes]
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-1),
  ATM_ARP(0x43, 0x23),
  ATM_NOTE_D3,
  ATM_DELAY(32),
  ATM_NOTE_A2,
  ATM_DELAY(32),
  ATM_NOTE_C3,
  ATM_DELAY(32),
  ATM_NOTE_G2,
  ATM_DELAY(32),
  ATM_NOTE_D3,
  ATM_DELAY(32),
  ATM_NOTE_A2,
  ATM_DELAY(32),
  ATM_NOTE_C3,
  ATM_DELAY(32),
  ATM_NOTE_G2,
  ATM_DELAY(32),
  ATM_RETURN,

  //"Track 10" bD1  [12 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D3,
  ATM_DELAY(2),
  ATM_NOTE_D4,
  ATM_DELAY(6),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 11" bD2  [15 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(2),
  ATM_VOL(0),
  ATM_DELAY(2),
  ATM_RETURN,

  //"Track 12" bA  [13 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(6),
  ATM_VOL(0),
  ATM_DELAY(2),
  ATM_RETURN,

  //"Track 13" bC1  [17 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(2),
  ATM_VOL(0),
  ATM_DELAY(2),
  ATM_VOL(56),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 14" bC2  [13 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(6),
  ATM_VOL(0),
  ATM_DELAY(2),
  ATM_RETURN,

  //"Track 15" bG1  [12 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_NOTE_D5,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 16" bG2  [11 bytes]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(56),
  ATM_NOTE_D4,
  ATM_DELAY(8),
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 17" bass  [7 bytes]
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(1, 18),   // 2x bass8
  ATM_GOTO(17),   // -> bass

  //"Track 18" bass8  [51 bytes]
  ATM_SET_TRA(0),
  ATM_GOTO(10),   // -> bD1
  ATM_GOTO(11),   // -> bD2
  ATM_SET_TRA((uint8_t)-5),
  ATM_GOTO(12),   // -> bA
  ATM_GOTO(12),   // -> bA
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(13),   // -> bC1
  ATM_GOTO(14),   // -> bC2
  ATM_SET_TRA((uint8_t)-7),
  ATM_GOTO(15),   // -> bG1
  ATM_GOTO(16),   // -> bG2
  ATM_SET_TRA(0),
  ATM_GOTO(10),   // -> bD1
  ATM_GOTO(11),   // -> bD2
  ATM_SET_TRA((uint8_t)-5),
  ATM_GOTO(12),   // -> bA
  ATM_GOTO(12),   // -> bA
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(13),   // -> bC1
  ATM_GOTO(14),   // -> bC2
  ATM_SET_TRA((uint8_t)-7),
  ATM_GOTO(15),   // -> bG1
  ATM_SET_TRA((uint8_t)-9),
  ATM_GOTO(16),   // -> bG2
  ATM_RETURN,

};

#endif
