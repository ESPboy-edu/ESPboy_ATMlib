#ifndef SONG_H
#define SONG_H

// ---------------------------------------------------------------------------
//  "Neon Heart" - Italo disco loop, 8 bars, E minor, ~135 BPM, ~14.2 s, loops seamlessly
//
//  Play with:   #include "newsong3.h"      ATM.play(neonHeart);
//
//  Timing : ATM_SET_TEMPO(36) (doubled).  1 beat = 16 ticks, 1 bar = 64 ticks, 8 bars = 512 ticks.
//           Every channel sums to exactly 512 ticks of delay and its entry
//           track ends with ATM_GOTO(itself): same-tick restart, no STOP, no hop.
//  Chords : Em | C | G | D | Em | C | D | B     (the B is the dominant: it pulls back to Em)
//  Channels (ATMlib defaults):
//     CH0 PULSE  - lead melody, one track per bar
//     CH1 SQUARE - pitch-dropping kick on every beat + off-beat arpeggio chord stabs
//                  (inversions Em, C/E, G/D, D/F#, B/D# keep the stabs in one register)
//     CH2 SAW    - ducked bass (silent on the kick, then root-octave-root), transposed per chord
//     CH3 NOISE  - kick click, clap on 2 & 4, closed/open hats, snare roll fills (bars 4 and 8)
//  Note   : track 0 is only CH3's entry point and is never called.
//  Pitch  : ATMlib's note names sound about an octave above their label (same for every ATMlib song).
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif



Song neonHeart[] = {     // total song bytes = 476
  0x1B,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0      0   drums
  0x0E, 0x00,                 // Address of track 1     14   lead
  0x24, 0x00,                 // Address of track 2     36   bar1
  0x35, 0x00,                 // Address of track 3     53   bar2
  0x46, 0x00,                 // Address of track 4     70   bar3
  0x55, 0x00,                 // Address of track 5     85   bar4
  0x64, 0x00,                 // Address of track 6    100   bar5
  0x75, 0x00,                 // Address of track 7    117   bar6
  0x86, 0x00,                 // Address of track 8    134   bar7
  0x97, 0x00,                 // Address of track 9    151   bar8
  0xA8, 0x00,                 // Address of track 10   168   chords
  0xC4, 0x00,                 // Address of track 11   196   kick
  0xCF, 0x00,                 // Address of track 12   207   beat_Em
  0xDB, 0x00,                 // Address of track 13   219   beat_C
  0xE7, 0x00,                 // Address of track 14   231   beat_G
  0xF3, 0x00,                 // Address of track 15   243   beat_D
  0xFF, 0x00,                 // Address of track 16   255   beat_B
  0x0B, 0x01,                 // Address of track 17   267   bass
  0x2F, 0x01,                 // Address of track 18   303   bass_beat
  0x3B, 0x01,                 // Address of track 19   315   bass_beat4
  0x47, 0x01,                 // Address of track 20   327   bass_bar
  0x4D, 0x01,                 // Address of track 21   333   drum_A
  0x5D, 0x01,                 // Address of track 22   349   drum_B
  0x68, 0x01,                 // Address of track 23   360   drum_pair
  0x6D, 0x01,                 // Address of track 24   365   drum_bar
  0x71, 0x01,                 // Address of track 25   369   drum_fill
  0x9A, 0x01,                 // Address of track 26   410   drum_bar_fill

  0x01,                         // CH0 entry -> track 1 (lead)
  0x0A,                         // CH1 entry -> track 10 (chords)
  0x11,                         // CH2 entry -> track 17 (bass)
  0x00,                         // CH3 entry -> track 0 (drums)

  //"Track 0" drums  [14b]
  ATM_SET_TEMPO(36),
  ATM_REPEAT(2, 24),   // 3x drum_bar
  ATM_GOTO(26),   // -> drum_bar_fill
  ATM_REPEAT(2, 24),   // 3x drum_bar
  ATM_GOTO(26),   // -> drum_bar_fill
  ATM_GOTO(0),   // loop: restart this track (no STOP, no gap)

  //"Track 1" lead  [22b]
  ATM_VOL(38),
  ATM_SL_VOL((uint8_t)-2),
  ATM_GOTO(2),   // -> bar1
  ATM_GOTO(3),   // -> bar2
  ATM_GOTO(4),   // -> bar3
  ATM_GOTO(5),   // -> bar4
  ATM_GOTO(6),   // -> bar5
  ATM_GOTO(7),   // -> bar6
  ATM_GOTO(8),   // -> bar7
  ATM_GOTO(9),   // -> bar8
  ATM_GOTO(1),   // loop: restart this track (no STOP, no gap)

  //"Track 2" bar1  [17b]
  ATM_NOTE_B5,
  ATM_DELAY(12),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(12),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 3" bar2  [17b]
  ATM_NOTE_G5,
  ATM_DELAY(12),
  ATM_NOTE_E5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(12),
  ATM_NOTE_G5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" bar3  [15b]
  ATM_NOTE_D6,
  ATM_DELAY(12),
  ATM_NOTE_B5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 5" bar4  [15b]
  ATM_NOTE_A5,
  ATM_DELAY(12),
  ATM_NOTE_F5_,
  ATM_DELAY(4),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_F5_,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 6" bar5  [17b]
  ATM_NOTE_E6,
  ATM_DELAY(12),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(12),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 7" bar6  [17b]
  ATM_NOTE_C6,
  ATM_DELAY(12),
  ATM_NOTE_B5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(12),
  ATM_NOTE_B5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 8" bar7  [17b]
  ATM_NOTE_F5_,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_F6_,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(12),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_F5_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 9" bar8  [17b]
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_D6_,
  ATM_DELAY(8),
  ATM_NOTE_F6_,
  ATM_DELAY(8),
  ATM_NOTE_D6_,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_F5_,
  ATM_DELAY(8),
  ATM_NOTE_D5_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 10" chords  [28b]
  ATM_REPEAT(3, 12),   // 4x beat_Em
  ATM_REPEAT(3, 13),   // 4x beat_C
  ATM_REPEAT(3, 14),   // 4x beat_G
  ATM_REPEAT(3, 15),   // 4x beat_D
  ATM_REPEAT(3, 12),   // 4x beat_Em
  ATM_REPEAT(3, 13),   // 4x beat_C
  ATM_REPEAT(3, 15),   // 4x beat_D
  ATM_REPEAT(2, 16),   // 3x beat_B
  ATM_GOTO(16),   // -> beat_B
  ATM_GOTO(10),   // loop: restart this track (no STOP, no gap)

  //"Track 11" kick  [11b]
  ATM_ARP_OFF,
  ATM_SL_FRQ((uint8_t)-127),
  ATM_VOL(30),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 12" beat_Em  [12b]
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_E3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 13" beat_C  [12b]
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_E3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 14" beat_G  [12b]
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x54, 0x20),
  ATM_NOTE_D3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 15" beat_D  [12b]
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_F3_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 16" beat_B  [12b]
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_D3_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 17" bass  [36b]
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-9),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA(0),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-5),
  ATM_GOTO(20),   // -> bass_bar
  ATM_GOTO(17),   // loop: restart this track (no STOP, no gap)

  //"Track 18" bass_beat  [12b]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(63),
  ATM_NOTE_E3,
  ATM_DELAY(4),
  ATM_NOTE_E4,
  ATM_DELAY(4),
  ATM_NOTE_E3,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 19" bass_beat4  [12b]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(63),
  ATM_NOTE_E3,
  ATM_DELAY(4),
  ATM_NOTE_E3,
  ATM_DELAY(4),
  ATM_NOTE_E4,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 20" bass_bar  [6b]
  ATM_REPEAT(2, 18),   // 3x bass_beat
  ATM_GOTO(19),   // -> bass_beat4
  ATM_RETURN,

  //"Track 21" drum_A  [16b]
  ATM_VOL(28),
  ATM_SL_VOL((uint8_t)-14),
  ATM_DELAY(4),
  ATM_VOL(12),
  ATM_SL_VOL((uint8_t)-6),
  ATM_DELAY(4),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 22" drum_B  [11b]
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(8),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 23" drum_pair  [5b]
  ATM_GOTO(21),   // -> drum_A
  ATM_GOTO(22),   // -> drum_B
  ATM_RETURN,

  //"Track 24" drum_bar  [4b]
  ATM_REPEAT(1, 23),   // 2x drum_pair
  ATM_RETURN,

  //"Track 25" drum_fill  [41b]
  ATM_VOL(22),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(38),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(42),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(46),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_VOL(50),
  ATM_SL_VOL((uint8_t)-9),
  ATM_DELAY(2),
  ATM_RETURN,

  //"Track 26" drum_bar_fill  [7b]
  ATM_GOTO(23),   // -> drum_pair
  ATM_GOTO(21),   // -> drum_A
  ATM_GOTO(25),   // -> drum_fill
  ATM_RETURN,

};

#endif
