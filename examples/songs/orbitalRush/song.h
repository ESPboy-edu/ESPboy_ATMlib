#ifndef ORBITALRUSH_H
#define ORBITALRUSH_H

// ---------------------------------------------------------------------------
//  "Orbital Rush" - an original demoscene style track for ATMlib           1479 bytes
//  (driving D minor, fast arpeggios, big melodic hooks, a key-change finale)
//
//  Play with:   #include "song.h"      ATM.play(orbitalRush);
//
//  NEVER ENDS: an 8-bar intro plays once, then the 96-bar body (about 2:46) loops forever.
//  The first pass is intro + body = 104 bars = about 3:00.
//
//  Timing : ATM_SET_TEMPO(37) (doubled) = about 139 BPM.  1 beat = 16 ticks, 1 bar = 64 ticks.
//  Loop   : hop-free self-loop (see LearnATMlib.md): every channel's entry track plays its intro once
//           and then calls its LOOP track, which calls the body and then calls ITSELF.
//           Every channel's body is exactly 6144 ticks (96 bars).  The body sets volume, slide,
//           arpeggio and transposition itself, so every pass is identical.
//           The riser glissando uses a tick amount that divides its length (a glissando counter
//           that does not divide it would carry over and shift the next pass by a tick).
//
//  FORM                                                            bars
//     INTRO (once)   pad fades in, drums build, lead + bass enter     8
//     A              Dm Bb F C x4   melody A1 A2 A1 A2, sparse bass  16
//     B              Gm Dm Bb C x4  melody B1 B2 B1 B2               16
//     break          Bb C Dm A x2   vibrato notes, pad, riser         8
//     chorus         Dm Bb Gm A x4  the big hook (C1 C2 C1 C2)       16
//     dance          Dm C Bb A x4   16th riff, four on the floor     16
//     lifted chorus  Em C Am B x4   same hook, +2 semitones (ADD_TRA)16
//     tag            Bb Gm C A x2   falls back into the A section     8
//
//  Channels (ATMlib defaults):
//     CH0 PULSE  - lead, absolute notes (the lift is ADD_TRA +2 around the chorus; the dance riff is
//                  transposed per chord)
//     CH1 SQUARE - arpeggio chords: pad (A, break, intro) and 3+3+2 stabs (everything else).
//                  Explicit roots (arpeggio + transposition on one channel would transpose twice).
//     CH2 SAW    - bass, ONE bar of 16ths transposed per chord (plus a sparse and a sustained pattern)
//     CH3 NOISE  - drums, four-on-the-floor in the dance, tremolo snare-roll fills
//  Note   : track 0 is only CH3's entry point and is never called from another track.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif



Song orbitalRush[] = {     // total song bytes = 1479
  0x5C,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0      0   drums
  0x06, 0x00,                 // Address of track 1      6   lead
  0x0A, 0x00,                 // Address of track 2     10   chords
  0x0E, 0x00,                 // Address of track 3     14   bass
  0x12, 0x00,                 // Address of track 4     18   lead_loop
  0x16, 0x00,                 // Address of track 5     22   chords_loop
  0x1A, 0x00,                 // Address of track 6     26   bass_loop
  0x1E, 0x00,                 // Address of track 7     30   drums_loop
  0x22, 0x00,                 // Address of track 8     34   A1
  0x51, 0x00,                 // Address of track 9     81   A2
  0x82, 0x00,                 // Address of track 10   130   B1
  0xA9, 0x00,                 // Address of track 11   169   B2
  0xD8, 0x00,                 // Address of track 12   216   K1
  0xE9, 0x00,                 // Address of track 13   233   K2
  0xFB, 0x00,                 // Address of track 14   251   C1
  0x26, 0x01,                 // Address of track 15   294   C2
  0x4D, 0x01,                 // Address of track 16   333   cell
  0x56, 0x01,                 // Address of track 17   342   cellB
  0x5F, 0x01,                 // Address of track 18   351   bar_d
  0x65, 0x01,                 // Address of track 19   357   cell2
  0x6E, 0x01,                 // Address of track 20   366   cell2b
  0x77, 0x01,                 // Address of track 21   375   bar_d2
  0x80, 0x01,                 // Address of track 22   384   dance_ph
  0x91, 0x01,                 // Address of track 23   401   dance2_ph
  0xA2, 0x01,                 // Address of track 24   418   T1
  0xBB, 0x01,                 // Address of track 25   443   T2
  0xE4, 0x01,                 // Address of track 26   484   A_pair
  0xE9, 0x01,                 // Address of track 27   489   B_pair
  0xEE, 0x01,                 // Address of track 28   494   C_pair
  0xF3, 0x01,                 // Address of track 29   499   l_intro
  0x00, 0x02,                 // Address of track 30   512   l_body
  0x41, 0x02,                 // Address of track 31   577   st_Gm
  0x51, 0x02,                 // Address of track 32   593   st_Dm
  0x61, 0x02,                 // Address of track 33   609   st_Bb
  0x71, 0x02,                 // Address of track 34   625   st_C
  0x81, 0x02,                 // Address of track 35   641   st_A
  0x91, 0x02,                 // Address of track 36   657   st_Em
  0xA1, 0x02,                 // Address of track 37   673   st_Am
  0xB1, 0x02,                 // Address of track 38   689   st_B
  0xC1, 0x02,                 // Address of track 39   705   pd_Dm
  0xC7, 0x02,                 // Address of track 40   711   pd_Bb
  0xCD, 0x02,                 // Address of track 41   717   pd_F
  0xD3, 0x02,                 // Address of track 42   723   pd_C
  0xD9, 0x02,                 // Address of track 43   729   pd_A
  0xDF, 0x02,                 // Address of track 44   735   ph_pd_A
  0xE8, 0x02,                 // Address of track 45   744   ph_pd_br
  0xF1, 0x02,                 // Address of track 46   753   ph_st_B
  0xFA, 0x02,                 // Address of track 47   762   ph_st_C
  0x03, 0x03,                 // Address of track 48   771   ph_st_D
  0x0C, 0x03,                 // Address of track 49   780   ph_st_L
  0x15, 0x03,                 // Address of track 50   789   ph_st_T
  0x1E, 0x03,                 // Address of track 51   798   c_intro
  0x28, 0x03,                 // Address of track 52   808   c_body
  0x4F, 0x03,                 // Address of track 53   847   bassS
  0x60, 0x03,                 // Address of track 54   864   bassP
  0x81, 0x03,                 // Address of track 55   897   bassB
  0x88, 0x03,                 // Address of track 56   904   bAs
  0x99, 0x03,                 // Address of track 57   921   bAp
  0xAA, 0x03,                 // Address of track 58   938   bB
  0xBD, 0x03,                 // Address of track 59   957   bC
  0xCE, 0x03,                 // Address of track 60   974   bD
  0xDF, 0x03,                 // Address of track 61   991   bK
  0xF2, 0x03,                 // Address of track 62  1010   bT
  0x05, 0x04,                 // Address of track 63  1029   b_intro
  0x12, 0x04,                 // Address of track 64  1042   b_body
  0x3A, 0x04,                 // Address of track 65  1082   k4
  0x40, 0x04,                 // Address of track 66  1088   s4
  0x46, 0x04,                 // Address of track 67  1094   h4
  0x4C, 0x04,                 // Address of track 68  1100   o4
  0x52, 0x04,                 // Address of track 69  1106   b1
  0x58, 0x04,                 // Address of track 70  1112   b2
  0x60, 0x04,                 // Address of track 71  1120   b3
  0x69, 0x04,                 // Address of track 72  1129   b4
  0x72, 0x04,                 // Address of track 73  1138   bS
  0x78, 0x04,                 // Address of track 74  1144   bhh
  0x7C, 0x04,                 // Address of track 75  1148   bf
  0x85, 0x04,                 // Address of track 76  1157   roll16
  0x8F, 0x04,                 // Address of track 77  1167   roll64
  0x9A, 0x04,                 // Address of track 78  1178   bar_g
  0xA3, 0x04,                 // Address of track 79  1187   bar_gf
  0xAC, 0x04,                 // Address of track 80  1196   bar_v
  0xB5, 0x04,                 // Address of track 81  1205   bar_vf
  0xBE, 0x04,                 // Address of track 82  1214   bar_hat
  0xC2, 0x04,                 // Address of track 83  1218   bar_df
  0xC6, 0x04,                 // Address of track 84  1222   bar_dfill
  0xCC, 0x04,                 // Address of track 85  1228   bar_br
  0xD2, 0x04,                 // Address of track 86  1234   dr4v
  0xD8, 0x04,                 // Address of track 87  1240   dr4g
  0xDE, 0x04,                 // Address of track 88  1246   dr4d
  0xE4, 0x04,                 // Address of track 89  1252   dr_br
  0xEC, 0x04,                 // Address of track 90  1260   d_intro
  0xF5, 0x04,                 // Address of track 91  1269   d_body

  0x01,                         // CH0 entry -> track 1 (lead)
  0x02,                         // CH1 entry -> track 2 (chords)
  0x03,                         // CH2 entry -> track 3 (bass)
  0x00,                         // CH3 entry -> track 0 (drums)

  //"Track 0" drums  [6b]
  ATM_SET_TEMPO(37),
  ATM_GOTO(90),   // -> d_intro
  ATM_GOTO(7),   // -> drums_loop

  //"Track 1" lead  [4b]
  ATM_GOTO(29),   // -> l_intro
  ATM_GOTO(4),   // -> lead_loop

  //"Track 2" chords  [4b]
  ATM_GOTO(51),   // -> c_intro
  ATM_GOTO(5),   // -> chords_loop

  //"Track 3" bass  [4b]
  ATM_GOTO(63),   // -> b_intro
  ATM_GOTO(6),   // -> bass_loop

  //"Track 4" lead_loop  [4b]
  ATM_GOTO(30),   // -> l_body
  ATM_GOTO(4),   // -> lead_loop

  //"Track 5" chords_loop  [4b]
  ATM_GOTO(52),   // -> c_body
  ATM_GOTO(5),   // -> chords_loop

  //"Track 6" bass_loop  [4b]
  ATM_GOTO(64),   // -> b_body
  ATM_GOTO(6),   // -> bass_loop

  //"Track 7" drums_loop  [4b]
  ATM_GOTO(91),   // -> d_body
  ATM_GOTO(7),   // -> drums_loop

  //"Track 8" A1  [47b]
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_F5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(24),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_F5,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_F6,
  ATM_DELAY(24),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 9" A2  [49b]
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 10" B1  [39b]
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_G6,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 11" B2  [47b]
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_G6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 12" K1  [17b]
  ATM_NOTE_D6,
  ATM_DELAY(48),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(48),
  ATM_NOTE_G6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(32),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 13" K2  [18b]
  ATM_NOTE_D6,
  ATM_DELAY(32),
  ATM_NOTE_F6,
  ATM_DELAY(32),
  ATM_NOTE_E6,
  ATM_DELAY(32),
  ATM_NOTE_G6,
  ATM_DELAY(32),
  ATM_NOTE_A6,
  ATM_DELAY(32),
  ATM_NOTE_F6,
  ATM_DELAY(32),
  ATM_NOTE_C4,
  ATM_GLIS(0x01),
  ATM_DELAY(64),
  ATM_GLIS_OFF,
  ATM_RETURN,

  //"Track 14" C1  [43b]
  ATM_NOTE_D6,
  ATM_DELAY(24),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(24),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_G6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_C6_,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6_,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 15" C2  [39b]
  ATM_NOTE_F6,
  ATM_DELAY(24),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(24),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_G6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(16),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_C6_,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 16" cell  [9b]
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 17" cellB  [9b]
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_E6,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 18" bar_d  [6b]
  ATM_REPEAT(2, 16),   // 3x cell
  ATM_GOTO(17),   // -> cellB
  ATM_RETURN,

  //"Track 19" cell2  [9b]
  ATM_NOTE_D5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 20" cell2b  [9b]
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_E6,
  ATM_DELAY(4),
  ATM_NOTE_D6,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 21" bar_d2  [9b]
  ATM_GOTO(19),   // -> cell2
  ATM_GOTO(20),   // -> cell2b
  ATM_GOTO(19),   // -> cell2
  ATM_GOTO(20),   // -> cell2b
  ATM_RETURN,

  //"Track 22" dance_ph  [17b]
  ATM_GOTO(18),   // -> bar_d
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(18),   // -> bar_d
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(18),   // -> bar_d
  ATM_ADD_TRA((uint8_t)-1),
  ATM_GOTO(18),   // -> bar_d
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 23" dance2_ph  [17b]
  ATM_GOTO(21),   // -> bar_d2
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(21),   // -> bar_d2
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(21),   // -> bar_d2
  ATM_ADD_TRA((uint8_t)-1),
  ATM_GOTO(21),   // -> bar_d2
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 24" T1  [25b]
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(32),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_A5_,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_C6_,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(32),
  ATM_RETURN,

  //"Track 25" T2  [41b]
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_A5_,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(32),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_NOTE_E6,
  ATM_DELAY(8),
  ATM_NOTE_C6_,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(32),
  ATM_RETURN,

  //"Track 26" A_pair  [5b]
  ATM_GOTO(8),   // -> A1
  ATM_GOTO(9),   // -> A2
  ATM_RETURN,

  //"Track 27" B_pair  [5b]
  ATM_GOTO(10),   // -> B1
  ATM_GOTO(11),   // -> B2
  ATM_RETURN,

  //"Track 28" C_pair  [5b]
  ATM_GOTO(14),   // -> C1
  ATM_GOTO(15),   // -> C2
  ATM_RETURN,

  //"Track 29" l_intro  [13b]
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_GOTO(8),   // -> A1
  ATM_RETURN,

  //"Track 30" l_body  [65b]
  ATM_SET_TRA(0),
  ATM_VOL(34),
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 26),   // 2x A_pair
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 27),   // 2x B_pair
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(3, 3),
  ATM_GOTO(12),   // -> K1
  ATM_GOTO(13),   // -> K2
  ATM_VIB_OFF,
  ATM_VOL(46),
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 28),   // 2x C_pair
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 22),   // 2x dance_ph
  ATM_REPEAT(1, 23),   // 2x dance2_ph
  ATM_VOL(46),
  ATM_SL_VOL((uint8_t)-1),
  ATM_ADD_TRA(2),
  ATM_REPEAT(1, 28),   // 2x C_pair
  ATM_ADD_TRA((uint8_t)-2),
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_GOTO(24),   // -> T1
  ATM_GOTO(25),   // -> T2
  ATM_RETURN,

  //"Track 31" st_Gm  [16b]
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_G3,
  ATM_DELAY(12),
  ATM_NOTE_G3,
  ATM_DELAY(12),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(12),
  ATM_NOTE_G3,
  ATM_DELAY(12),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 32" st_Dm  [16b]
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_D4,
  ATM_DELAY(12),
  ATM_NOTE_D4,
  ATM_DELAY(12),
  ATM_NOTE_D4,
  ATM_DELAY(8),
  ATM_NOTE_D4,
  ATM_DELAY(12),
  ATM_NOTE_D4,
  ATM_DELAY(12),
  ATM_NOTE_D4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 33" st_Bb  [16b]
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_A3_,
  ATM_DELAY(12),
  ATM_NOTE_A3_,
  ATM_DELAY(12),
  ATM_NOTE_A3_,
  ATM_DELAY(8),
  ATM_NOTE_A3_,
  ATM_DELAY(12),
  ATM_NOTE_A3_,
  ATM_DELAY(12),
  ATM_NOTE_A3_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 34" st_C  [16b]
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_C4,
  ATM_DELAY(12),
  ATM_NOTE_C4,
  ATM_DELAY(12),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_C4,
  ATM_DELAY(12),
  ATM_NOTE_C4,
  ATM_DELAY(12),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 35" st_A  [16b]
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 36" st_Em  [16b]
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_E4,
  ATM_DELAY(12),
  ATM_NOTE_E4,
  ATM_DELAY(12),
  ATM_NOTE_E4,
  ATM_DELAY(8),
  ATM_NOTE_E4,
  ATM_DELAY(12),
  ATM_NOTE_E4,
  ATM_DELAY(12),
  ATM_NOTE_E4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 37" st_Am  [16b]
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(12),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 38" st_B  [16b]
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_B3,
  ATM_DELAY(12),
  ATM_NOTE_B3,
  ATM_DELAY(12),
  ATM_NOTE_B3,
  ATM_DELAY(8),
  ATM_NOTE_B3,
  ATM_DELAY(12),
  ATM_NOTE_B3,
  ATM_DELAY(12),
  ATM_NOTE_B3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 39" pd_Dm  [6b]
  ATM_ARP(0x34, 0x22),
  ATM_NOTE_D4,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 40" pd_Bb  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_A3_,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 41" pd_F  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_F4,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 42" pd_C  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_C4,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 43" pd_A  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_A3,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 44" ph_pd_A  [9b]
  ATM_GOTO(39),   // -> pd_Dm
  ATM_GOTO(40),   // -> pd_Bb
  ATM_GOTO(41),   // -> pd_F
  ATM_GOTO(42),   // -> pd_C
  ATM_RETURN,

  //"Track 45" ph_pd_br  [9b]
  ATM_GOTO(40),   // -> pd_Bb
  ATM_GOTO(42),   // -> pd_C
  ATM_GOTO(39),   // -> pd_Dm
  ATM_GOTO(43),   // -> pd_A
  ATM_RETURN,

  //"Track 46" ph_st_B  [9b]
  ATM_GOTO(31),   // -> st_Gm
  ATM_GOTO(32),   // -> st_Dm
  ATM_GOTO(33),   // -> st_Bb
  ATM_GOTO(34),   // -> st_C
  ATM_RETURN,

  //"Track 47" ph_st_C  [9b]
  ATM_GOTO(32),   // -> st_Dm
  ATM_GOTO(33),   // -> st_Bb
  ATM_GOTO(31),   // -> st_Gm
  ATM_GOTO(35),   // -> st_A
  ATM_RETURN,

  //"Track 48" ph_st_D  [9b]
  ATM_GOTO(32),   // -> st_Dm
  ATM_GOTO(34),   // -> st_C
  ATM_GOTO(33),   // -> st_Bb
  ATM_GOTO(35),   // -> st_A
  ATM_RETURN,

  //"Track 49" ph_st_L  [9b]
  ATM_GOTO(36),   // -> st_Em
  ATM_GOTO(34),   // -> st_C
  ATM_GOTO(37),   // -> st_Am
  ATM_GOTO(38),   // -> st_B
  ATM_RETURN,

  //"Track 50" ph_st_T  [9b]
  ATM_GOTO(33),   // -> st_Bb
  ATM_GOTO(31),   // -> st_Gm
  ATM_GOTO(34),   // -> st_C
  ATM_GOTO(35),   // -> st_A
  ATM_RETURN,

  //"Track 51" c_intro  [10b]
  ATM_VOL(0),
  ATM_SL_VOL_ADV(1, 17),
  ATM_GOTO(44),   // -> ph_pd_A
  ATM_GOTO(44),   // -> ph_pd_A
  ATM_RETURN,

  //"Track 52" c_body  [39b]
  ATM_VOL(16),
  ATM_SL_VOL(0),
  ATM_REPEAT(3, 44),   // 4x ph_pd_A
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(3, 46),   // 4x ph_st_B
  ATM_VOL(16),
  ATM_SL_VOL(0),
  ATM_REPEAT(1, 45),   // 2x ph_pd_br
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(3, 47),   // 4x ph_st_C
  ATM_REPEAT(3, 48),   // 4x ph_st_D
  ATM_REPEAT(3, 49),   // 4x ph_st_L
  ATM_GOTO(50),   // -> ph_st_T
  ATM_GOTO(50),   // -> ph_st_T
  ATM_RETURN,

  //"Track 53" bassS  [17b]
  ATM_NOTE_D3,
  ATM_DELAY(12),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_NOTE_D3,
  ATM_DELAY(8),
  ATM_NOTE_D3,
  ATM_DELAY(12),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(8),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 54" bassP  [33b]
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_D4,
  ATM_DELAY(4),
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 55" bassB  [7b]
  ATM_NOTE_D3,
  ATM_DELAY(32),
  ATM_NOTE_A3,
  ATM_DELAY(16),
  ATM_NOTE_D3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 56" bAs  [17b]
  ATM_GOTO(53),   // -> bassS
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(53),   // -> bassS
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(53),   // -> bassS
  ATM_ADD_TRA(7),
  ATM_GOTO(53),   // -> bassS
  ATM_ADD_TRA(2),
  ATM_RETURN,

  //"Track 57" bAp  [17b]
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(7),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(2),
  ATM_RETURN,

  //"Track 58" bB  [19b]
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(7),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(2),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(2),
  ATM_RETURN,

  //"Track 59" bC  [17b]
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(2),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 60" bD  [17b]
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-1),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 61" bK  [19b]
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(55),   // -> bassB
  ATM_ADD_TRA(2),
  ATM_GOTO(55),   // -> bassB
  ATM_ADD_TRA(2),
  ATM_GOTO(55),   // -> bassB
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(55),   // -> bassB
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 62" bT  [19b]
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(5),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(54),   // -> bassP
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 63" b_intro  [13b]
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_VOL(63),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_GOTO(57),   // -> bAp
  ATM_RETURN,

  //"Track 64" b_body  [40b]
  ATM_SET_TRA(0),
  ATM_VOL(63),
  ATM_SL_VOL((uint8_t)-3),
  ATM_REPEAT(1, 56),   // 2x bAs
  ATM_REPEAT(1, 57),   // 2x bAp
  ATM_REPEAT(3, 58),   // 4x bB
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 61),   // 2x bK
  ATM_SL_VOL((uint8_t)-3),
  ATM_REPEAT(3, 59),   // 4x bC
  ATM_REPEAT(3, 60),   // 4x bD
  ATM_ADD_TRA(2),
  ATM_REPEAT(3, 59),   // 4x bC
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(62),   // -> bT
  ATM_GOTO(62),   // -> bT
  ATM_RETURN,

  //"Track 65" k4  [6b]
  ATM_VOL(48),
  ATM_SL_VOL((uint8_t)-12),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 66" s4  [6b]
  ATM_VOL(44),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 67" h4  [6b]
  ATM_VOL(14),
  ATM_SL_VOL((uint8_t)-7),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 68" o4  [6b]
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 69" b1  [6b]
  ATM_GOTO(65),   // -> k4
  ATM_REPEAT(2, 67),   // 3x h4
  ATM_RETURN,

  //"Track 70" b2  [8b]
  ATM_GOTO(66),   // -> s4
  ATM_REPEAT(1, 67),   // 2x h4
  ATM_GOTO(65),   // -> k4
  ATM_RETURN,

  //"Track 71" b3  [9b]
  ATM_GOTO(65),   // -> k4
  ATM_GOTO(67),   // -> h4
  ATM_GOTO(68),   // -> o4
  ATM_GOTO(67),   // -> h4
  ATM_RETURN,

  //"Track 72" b4  [9b]
  ATM_GOTO(66),   // -> s4
  ATM_GOTO(67),   // -> h4
  ATM_GOTO(65),   // -> k4
  ATM_GOTO(67),   // -> h4
  ATM_RETURN,

  //"Track 73" bS  [6b]
  ATM_GOTO(66),   // -> s4
  ATM_REPEAT(2, 67),   // 3x h4
  ATM_RETURN,

  //"Track 74" bhh  [4b]
  ATM_REPEAT(3, 67),   // 4x h4
  ATM_RETURN,

  //"Track 75" bf  [9b]
  ATM_GOTO(65),   // -> k4
  ATM_GOTO(67),   // -> h4
  ATM_GOTO(65),   // -> k4
  ATM_GOTO(67),   // -> h4
  ATM_RETURN,

  //"Track 76" roll16  [10b]
  ATM_VOL(10),
  ATM_SL_VOL(2),
  ATM_TREM(6, 0),
  ATM_DELAY(16),
  ATM_TREM_OFF,
  ATM_RETURN,

  //"Track 77" roll64  [11b]
  ATM_VOL(10),
  ATM_SL_VOL_ADV(1, 1),
  ATM_TREM(6, 0),
  ATM_DELAY(64),
  ATM_TREM_OFF,
  ATM_RETURN,

  //"Track 78" bar_g  [9b]
  ATM_GOTO(69),   // -> b1
  ATM_GOTO(70),   // -> b2
  ATM_GOTO(71),   // -> b3
  ATM_GOTO(72),   // -> b4
  ATM_RETURN,

  //"Track 79" bar_gf  [9b]
  ATM_GOTO(69),   // -> b1
  ATM_GOTO(70),   // -> b2
  ATM_GOTO(71),   // -> b3
  ATM_GOTO(76),   // -> roll16
  ATM_RETURN,

  //"Track 80" bar_v  [9b]
  ATM_GOTO(69),   // -> b1
  ATM_GOTO(73),   // -> bS
  ATM_GOTO(69),   // -> b1
  ATM_GOTO(73),   // -> bS
  ATM_RETURN,

  //"Track 81" bar_vf  [9b]
  ATM_GOTO(69),   // -> b1
  ATM_GOTO(73),   // -> bS
  ATM_GOTO(69),   // -> b1
  ATM_GOTO(76),   // -> roll16
  ATM_RETURN,

  //"Track 82" bar_hat  [4b]
  ATM_REPEAT(3, 74),   // 4x bhh
  ATM_RETURN,

  //"Track 83" bar_df  [4b]
  ATM_REPEAT(3, 75),   // 4x bf
  ATM_RETURN,

  //"Track 84" bar_dfill  [6b]
  ATM_REPEAT(2, 75),   // 3x bf
  ATM_GOTO(76),   // -> roll16
  ATM_RETURN,

  //"Track 85" bar_br  [6b]
  ATM_GOTO(69),   // -> b1
  ATM_REPEAT(2, 74),   // 3x bhh
  ATM_RETURN,

  //"Track 86" dr4v  [6b]
  ATM_REPEAT(2, 80),   // 3x bar_v
  ATM_GOTO(81),   // -> bar_vf
  ATM_RETURN,

  //"Track 87" dr4g  [6b]
  ATM_REPEAT(2, 78),   // 3x bar_g
  ATM_GOTO(79),   // -> bar_gf
  ATM_RETURN,

  //"Track 88" dr4d  [6b]
  ATM_REPEAT(2, 83),   // 3x bar_df
  ATM_GOTO(84),   // -> bar_dfill
  ATM_RETURN,

  //"Track 89" dr_br  [8b]
  ATM_REPEAT(5, 85),   // 6x bar_br
  ATM_GOTO(78),   // -> bar_g
  ATM_GOTO(77),   // -> roll64
  ATM_RETURN,

  //"Track 90" d_intro  [9b]
  ATM_REPEAT(3, 82),   // 4x bar_hat
  ATM_REPEAT(2, 78),   // 3x bar_g
  ATM_GOTO(79),   // -> bar_gf
  ATM_RETURN,

  //"Track 91" d_body  [21b]
  ATM_REPEAT(3, 86),   // 4x dr4v
  ATM_REPEAT(3, 87),   // 4x dr4g
  ATM_GOTO(89),   // -> dr_br
  ATM_REPEAT(3, 87),   // 4x dr4g
  ATM_REPEAT(3, 88),   // 4x dr4d
  ATM_REPEAT(3, 87),   // 4x dr4g
  ATM_REPEAT(1, 87),   // 2x dr4g
  ATM_RETURN,

};

#endif
