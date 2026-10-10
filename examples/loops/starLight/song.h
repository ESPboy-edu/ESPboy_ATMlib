#ifndef STARLIGHT_H
#define STARLIGHT_H

// ---------------------------------------------------------------------------
//  "Star Light" - an original K-pop style dance track for ATMlib          1307 bytes
//
//  Play with:   #include "starlight.h"      ATM.play(starLight);
//
//  NEVER ENDS: an 8-bar intro plays once, then the 84-bar body (about 2:50) loops forever.
//  The first pass is intro + body = 92 bars = about 3:00.  There is no ending.
//
//  Timing : ATM_SET_TEMPO(33) (doubled) = about 124 BPM.  1 beat = 16 ticks, 1 bar = 64 ticks.
//  Loop   : every channel's entry track plays the intro once and then calls its LOOP track, which
//           calls the body and then calls ITSELF (ATM_GOTO to the track it is already in).  That
//           restarts in the same tick: no STOP, no ATM_GOTO_ADV, no silent tick, no hop.
//           Every channel's body is exactly 5376 ticks (84 bars) of delay.
//           The body sets volume, slide, arpeggio and transposition itself, so every pass is identical.
//
//  FORM                                                            bars
//     INTRO (once)   pad fades in, hook teaser, groove builds, roll   8
//     verse 1        Am F C G x2   (absolute melody, sparse bass)     8
//     pre-chorus 1   F G F G       (one rising motif, transposed)     4
//     chorus 1       C G Am F x2   (hook transposed per chord)        8
//     dance break 1  Am F C G      (16th riff, transposed per chord)  4
//     verse 2        same, melody an octave higher (ADD_TRA 12)       8
//     pre-chorus 2                                                    4
//     chorus 2                                                        8
//     dance break 2                                                   4
//     bridge         F G Em Am x2  (vibrato, slow pad, riser)         8
//     verse 3                                                         8
//     pre-chorus 3                                                    4
//     last chorus    D A Bm G x4   (key change: a whole step up)     16
//                    ends on G, which is chord V of the verse key: it falls back into Am
//
//  Channels (ATMlib defaults):
//     CH0 PULSE  - lead.  The hook, the pre-chorus motif and the dance riff are ONE track each
//                  and are transposed per chord with ATM_ADD_TRA; the key change is SET at the start.
//     CH1 SQUARE - arpeggio chords: slow pad (verse, bridge, intro) and 3+3+2 stabs (pre, chorus,
//                  dance).  Explicit roots (arpeggio + transposition on one channel would transpose twice).
//     CH2 SAW    - bass, transposed per chord, three patterns (sparse, driving, sustained)
//     CH3 NOISE  - dance groove, four-on-the-floor in the dance breaks, tremolo snare-roll fills
//  Note   : track 0 is only CH3's entry point and is never called from another track.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif



Song starLight[] = {     // total song bytes = 1307
  0x5A,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0      0   drums
  0x06, 0x00,                 // Address of track 1      6   lead
  0x0A, 0x00,                 // Address of track 2     10   chords
  0x0E, 0x00,                 // Address of track 3     14   bass
  0x12, 0x00,                 // Address of track 4     18   lead_loop
  0x16, 0x00,                 // Address of track 5     22   chords_loop
  0x1A, 0x00,                 // Address of track 6     26   bass_loop
  0x1E, 0x00,                 // Address of track 7     30   drums_loop
  0x22, 0x00,                 // Address of track 8     34   hookA
  0x33, 0x00,                 // Address of track 9     51   hookB
  0x44, 0x00,                 // Address of track 10    68   hookC
  0x51, 0x00,                 // Address of track 11    81   hookE
  0x5E, 0x00,                 // Address of track 12    94   ch_ph1
  0x6F, 0x00,                 // Address of track 13   111   ch_ph2
  0x80, 0x00,                 // Address of track 14   128   vA
  0x93, 0x00,                 // Address of track 15   147   vB
  0xA6, 0x00,                 // Address of track 16   166   vC
  0xB9, 0x00,                 // Address of track 17   185   vD
  0xCC, 0x00,                 // Address of track 18   204   v_ph
  0xD5, 0x00,                 // Address of track 19   213   pm
  0xE4, 0x00,                 // Address of track 20   228   pmEnd
  0xEF, 0x00,                 // Address of track 21   239   pre_ph
  0x00, 0x01,                 // Address of track 22   256   cell
  0x09, 0x01,                 // Address of track 23   265   cellB
  0x12, 0x01,                 // Address of track 24   274   bar_d
  0x18, 0x01,                 // Address of track 25   280   dance_ph
  0x29, 0x01,                 // Address of track 26   297   br_a
  0x38, 0x01,                 // Address of track 27   312   br_b
  0x51, 0x01,                 // Address of track 28   337   l_intro
  0x5E, 0x01,                 // Address of track 29   350   ch_pair
  0x63, 0x01,                 // Address of track 30   355   l_body
  0xB2, 0x01,                 // Address of track 31   434   st_C
  0xC2, 0x01,                 // Address of track 32   450   st_G
  0xD2, 0x01,                 // Address of track 33   466   st_Am
  0xE2, 0x01,                 // Address of track 34   482   st_F
  0xF2, 0x01,                 // Address of track 35   498   st_D
  0x02, 0x02,                 // Address of track 36   514   st_A
  0x12, 0x02,                 // Address of track 37   530   st_Bm
  0x22, 0x02,                 // Address of track 38   546   pd_C
  0x28, 0x02,                 // Address of track 39   552   pd_G
  0x2E, 0x02,                 // Address of track 40   558   pd_Am
  0x34, 0x02,                 // Address of track 41   564   pd_F
  0x3A, 0x02,                 // Address of track 42   570   pd_Em
  0x40, 0x02,                 // Address of track 43   576   ph_pd_intro
  0x49, 0x02,                 // Address of track 44   585   ph_pd_verse
  0x52, 0x02,                 // Address of track 45   594   ph_pd_bridge
  0x5B, 0x02,                 // Address of track 46   603   ph_st_pre
  0x64, 0x02,                 // Address of track 47   612   ph_st_chor
  0x6D, 0x02,                 // Address of track 48   621   ph_st_dance
  0x76, 0x02,                 // Address of track 49   630   ph_st_lift
  0x7F, 0x02,                 // Address of track 50   639   c_intro
  0x89, 0x02,                 // Address of track 51   649   c_body
  0xC1, 0x02,                 // Address of track 52   705   bassV
  0xD2, 0x02,                 // Address of track 53   722   bassC
  0xE3, 0x02,                 // Address of track 54   739   bassB
  0xEA, 0x02,                 // Address of track 55   746   bv_ph
  0xFD, 0x02,                 // Address of track 56   765   bp_ph
  0x10, 0x03,                 // Address of track 57   784   bc_ph
  0x21, 0x03,                 // Address of track 58   801   bd_ph
  0x34, 0x03,                 // Address of track 59   820   bb_ph
  0x47, 0x03,                 // Address of track 60   839   b_intro
  0x54, 0x03,                 // Address of track 61   852   b_body
  0x82, 0x03,                 // Address of track 62   898   k4
  0x88, 0x03,                 // Address of track 63   904   s4
  0x8E, 0x03,                 // Address of track 64   910   h4
  0x94, 0x03,                 // Address of track 65   916   o4
  0x9A, 0x03,                 // Address of track 66   922   b1
  0xA0, 0x03,                 // Address of track 67   928   b2
  0xA8, 0x03,                 // Address of track 68   936   b3
  0xB1, 0x03,                 // Address of track 69   945   b4
  0xBA, 0x03,                 // Address of track 70   954   bS
  0xC0, 0x03,                 // Address of track 71   960   bhh
  0xC4, 0x03,                 // Address of track 72   964   bf
  0xCD, 0x03,                 // Address of track 73   973   roll16
  0xD7, 0x03,                 // Address of track 74   983   roll64
  0xE2, 0x03,                 // Address of track 75   994   bar_g
  0xEB, 0x03,                 // Address of track 76  1003   bar_gf
  0xF4, 0x03,                 // Address of track 77  1012   bar_v
  0xFD, 0x03,                 // Address of track 78  1021   bar_vf
  0x06, 0x04,                 // Address of track 79  1030   bar_hat
  0x0A, 0x04,                 // Address of track 80  1034   bar_df
  0x0E, 0x04,                 // Address of track 81  1038   bar_dfill
  0x14, 0x04,                 // Address of track 82  1044   bar_br
  0x1A, 0x04,                 // Address of track 83  1050   dr4v
  0x20, 0x04,                 // Address of track 84  1056   dr4g
  0x26, 0x04,                 // Address of track 85  1062   dr4d
  0x2C, 0x04,                 // Address of track 86  1068   dr_pre
  0x32, 0x04,                 // Address of track 87  1074   dr_br
  0x3A, 0x04,                 // Address of track 88  1082   d_intro
  0x43, 0x04,                 // Address of track 89  1091   d_body

  0x01,                         // CH0 entry -> track 1 (lead)
  0x02,                         // CH1 entry -> track 2 (chords)
  0x03,                         // CH2 entry -> track 3 (bass)
  0x00,                         // CH3 entry -> track 0 (drums)

  //"Track 0" drums  [6b]
  ATM_SET_TEMPO(33),
  ATM_GOTO(88),   // -> d_intro
  ATM_GOTO(7),   // -> drums_loop

  //"Track 1" lead  [4b]
  ATM_GOTO(28),   // -> l_intro
  ATM_GOTO(4),   // -> lead_loop

  //"Track 2" chords  [4b]
  ATM_GOTO(50),   // -> c_intro
  ATM_GOTO(5),   // -> chords_loop

  //"Track 3" bass  [4b]
  ATM_GOTO(60),   // -> b_intro
  ATM_GOTO(6),   // -> bass_loop

  //"Track 4" lead_loop  [4b]
  ATM_GOTO(30),   // -> l_body
  ATM_GOTO(4),   // -> lead_loop

  //"Track 5" chords_loop  [4b]
  ATM_GOTO(51),   // -> c_body
  ATM_GOTO(5),   // -> chords_loop

  //"Track 6" bass_loop  [4b]
  ATM_GOTO(61),   // -> b_body
  ATM_GOTO(6),   // -> bass_loop

  //"Track 7" drums_loop  [4b]
  ATM_GOTO(89),   // -> d_body
  ATM_GOTO(7),   // -> drums_loop

  //"Track 8" hookA  [17b]
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 9" hookB  [17b]
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_C6,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 10" hookC  [13b]
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 11" hookE  [13b]
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(16),
  ATM_NOTE_D5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 12" ch_ph1  [17b]
  ATM_GOTO(8),   // -> hookA
  ATM_ADD_TRA(7),
  ATM_GOTO(8),   // -> hookA
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(9),   // -> hookB
  ATM_ADD_TRA(5),
  ATM_GOTO(10),   // -> hookC
  ATM_ADD_TRA((uint8_t)-5),
  ATM_RETURN,

  //"Track 13" ch_ph2  [17b]
  ATM_GOTO(8),   // -> hookA
  ATM_ADD_TRA(7),
  ATM_GOTO(8),   // -> hookA
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(9),   // -> hookB
  ATM_ADD_TRA(5),
  ATM_GOTO(11),   // -> hookE
  ATM_ADD_TRA((uint8_t)-5),
  ATM_RETURN,

  //"Track 14" vA  [19b]
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(4),
  ATM_NOTE_A4,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 15" vB  [19b]
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(4),
  ATM_NOTE_C5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 16" vC  [19b]
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(4),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 17" vD  [19b]
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(4),
  ATM_NOTE_D5,
  ATM_DELAY(4),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 18" v_ph  [9b]
  ATM_GOTO(14),   // -> vA
  ATM_GOTO(15),   // -> vB
  ATM_GOTO(16),   // -> vC
  ATM_GOTO(17),   // -> vD
  ATM_RETURN,

  //"Track 19" pm  [15b]
  ATM_NOTE_F5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 20" pmEnd  [11b]
  ATM_NOTE_F5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_D6,
  ATM_DELAY(32),
  ATM_RETURN,

  //"Track 21" pre_ph  [17b]
  ATM_GOTO(19),   // -> pm
  ATM_ADD_TRA(2),
  ATM_GOTO(19),   // -> pm
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(19),   // -> pm
  ATM_ADD_TRA(2),
  ATM_GOTO(20),   // -> pmEnd
  ATM_ADD_TRA((uint8_t)-2),
  ATM_RETURN,

  //"Track 22" cell  [9b]
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 23" cellB  [9b]
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(4),
  ATM_NOTE_B5,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 24" bar_d  [6b]
  ATM_REPEAT(2, 22),   // 3x cell
  ATM_GOTO(23),   // -> cellB
  ATM_RETURN,

  //"Track 25" dance_ph  [17b]
  ATM_GOTO(24),   // -> bar_d
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(24),   // -> bar_d
  ATM_ADD_TRA(7),
  ATM_GOTO(24),   // -> bar_d
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(24),   // -> bar_d
  ATM_ADD_TRA(2),
  ATM_RETURN,

  //"Track 26" br_a  [15b]
  ATM_NOTE_C6,
  ATM_DELAY(32),
  ATM_NOTE_A5,
  ATM_DELAY(32),
  ATM_NOTE_B5,
  ATM_DELAY(32),
  ATM_NOTE_D6,
  ATM_DELAY(32),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_NOTE_B5,
  ATM_DELAY(32),
  ATM_NOTE_A5,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 27" br_b  [25b]
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_C6,
  ATM_DELAY(16),
  ATM_NOTE_F6,
  ATM_DELAY(32),
  ATM_NOTE_D6,
  ATM_DELAY(16),
  ATM_NOTE_B5,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_NOTE_B5,
  ATM_DELAY(16),
  ATM_NOTE_E6,
  ATM_DELAY(16),
  ATM_NOTE_G6,
  ATM_DELAY(32),
  ATM_NOTE_E5,
  ATM_GLIS(0x01),
  ATM_DELAY(32),
  ATM_GLIS_OFF,
  ATM_DELAY(32),
  ATM_RETURN,

  //"Track 28" l_intro  [13b]
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_GOTO(12),   // -> ch_ph1
  ATM_RETURN,

  //"Track 29" ch_pair  [5b]
  ATM_GOTO(12),   // -> ch_ph1
  ATM_GOTO(13),   // -> ch_ph2
  ATM_RETURN,

  //"Track 30" l_body  [79b]
  ATM_SET_TRA(0),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 18),   // 2x v_ph
  ATM_VOL(40),
  ATM_GOTO(21),   // -> pre_ph
  ATM_VOL(46),
  ATM_GOTO(12),   // -> ch_ph1
  ATM_GOTO(13),   // -> ch_ph2
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_GOTO(25),   // -> dance_ph
  ATM_VOL(36),
  ATM_ADD_TRA(12),
  ATM_REPEAT(1, 18),   // 2x v_ph
  ATM_ADD_TRA((uint8_t)-12),
  ATM_VOL(40),
  ATM_GOTO(21),   // -> pre_ph
  ATM_VOL(46),
  ATM_GOTO(12),   // -> ch_ph1
  ATM_GOTO(13),   // -> ch_ph2
  ATM_VOL(40),
  ATM_GOTO(25),   // -> dance_ph
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(3, 3),
  ATM_GOTO(26),   // -> br_a
  ATM_GOTO(27),   // -> br_b
  ATM_VIB_OFF,
  ATM_VOL(34),
  ATM_REPEAT(1, 18),   // 2x v_ph
  ATM_VOL(40),
  ATM_GOTO(21),   // -> pre_ph
  ATM_VOL(46),
  ATM_ADD_TRA(2),
  ATM_REPEAT(1, 29),   // 2x ch_pair
  ATM_ADD_TRA((uint8_t)-2),
  ATM_RETURN,

  //"Track 31" st_C  [16b]
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

  //"Track 32" st_G  [16b]
  ATM_ARP(0x43, 0x20),
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

  //"Track 33" st_Am  [16b]
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

  //"Track 34" st_F  [16b]
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_F3,
  ATM_DELAY(12),
  ATM_NOTE_F3,
  ATM_DELAY(12),
  ATM_NOTE_F3,
  ATM_DELAY(8),
  ATM_NOTE_F3,
  ATM_DELAY(12),
  ATM_NOTE_F3,
  ATM_DELAY(12),
  ATM_NOTE_F3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 35" st_D  [16b]
  ATM_ARP(0x43, 0x20),
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

  //"Track 36" st_A  [16b]
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

  //"Track 37" st_Bm  [16b]
  ATM_ARP(0x34, 0x20),
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

  //"Track 38" pd_C  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_C4,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 39" pd_G  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_G3,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 40" pd_Am  [6b]
  ATM_ARP(0x34, 0x22),
  ATM_NOTE_A3,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 41" pd_F  [6b]
  ATM_ARP(0x43, 0x22),
  ATM_NOTE_F3,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 42" pd_Em  [6b]
  ATM_ARP(0x34, 0x22),
  ATM_NOTE_E3,
  ATM_DELAY(64),
  ATM_RETURN,

  //"Track 43" ph_pd_intro  [9b]
  ATM_GOTO(38),   // -> pd_C
  ATM_GOTO(39),   // -> pd_G
  ATM_GOTO(40),   // -> pd_Am
  ATM_GOTO(41),   // -> pd_F
  ATM_RETURN,

  //"Track 44" ph_pd_verse  [9b]
  ATM_GOTO(40),   // -> pd_Am
  ATM_GOTO(41),   // -> pd_F
  ATM_GOTO(38),   // -> pd_C
  ATM_GOTO(39),   // -> pd_G
  ATM_RETURN,

  //"Track 45" ph_pd_bridge  [9b]
  ATM_GOTO(41),   // -> pd_F
  ATM_GOTO(39),   // -> pd_G
  ATM_GOTO(42),   // -> pd_Em
  ATM_GOTO(40),   // -> pd_Am
  ATM_RETURN,

  //"Track 46" ph_st_pre  [9b]
  ATM_GOTO(34),   // -> st_F
  ATM_GOTO(32),   // -> st_G
  ATM_GOTO(34),   // -> st_F
  ATM_GOTO(32),   // -> st_G
  ATM_RETURN,

  //"Track 47" ph_st_chor  [9b]
  ATM_GOTO(31),   // -> st_C
  ATM_GOTO(32),   // -> st_G
  ATM_GOTO(33),   // -> st_Am
  ATM_GOTO(34),   // -> st_F
  ATM_RETURN,

  //"Track 48" ph_st_dance  [9b]
  ATM_GOTO(33),   // -> st_Am
  ATM_GOTO(34),   // -> st_F
  ATM_GOTO(31),   // -> st_C
  ATM_GOTO(32),   // -> st_G
  ATM_RETURN,

  //"Track 49" ph_st_lift  [9b]
  ATM_GOTO(35),   // -> st_D
  ATM_GOTO(36),   // -> st_A
  ATM_GOTO(37),   // -> st_Bm
  ATM_GOTO(32),   // -> st_G
  ATM_RETURN,

  //"Track 50" c_intro  [10b]
  ATM_VOL(0),
  ATM_SL_VOL_ADV(1, 17),
  ATM_GOTO(43),   // -> ph_pd_intro
  ATM_GOTO(43),   // -> ph_pd_intro
  ATM_RETURN,

  //"Track 51" c_body  [56b]
  ATM_VOL(16),
  ATM_SL_VOL(0),
  ATM_REPEAT(1, 44),   // 2x ph_pd_verse
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-2),
  ATM_GOTO(46),   // -> ph_st_pre
  ATM_REPEAT(1, 47),   // 2x ph_st_chor
  ATM_GOTO(48),   // -> ph_st_dance
  ATM_VOL(16),
  ATM_SL_VOL(0),
  ATM_REPEAT(1, 44),   // 2x ph_pd_verse
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-2),
  ATM_GOTO(46),   // -> ph_st_pre
  ATM_REPEAT(1, 47),   // 2x ph_st_chor
  ATM_GOTO(48),   // -> ph_st_dance
  ATM_VOL(16),
  ATM_SL_VOL(0),
  ATM_REPEAT(1, 45),   // 2x ph_pd_bridge
  ATM_REPEAT(1, 44),   // 2x ph_pd_verse
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-2),
  ATM_GOTO(46),   // -> ph_st_pre
  ATM_REPEAT(3, 49),   // 4x ph_st_lift
  ATM_RETURN,

  //"Track 52" bassV  [17b]
  ATM_NOTE_C3,
  ATM_DELAY(12),
  ATM_NOTE_C3,
  ATM_DELAY(4),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(12),
  ATM_NOTE_C3,
  ATM_DELAY(4),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 53" bassC  [17b]
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 54" bassB  [7b]
  ATM_NOTE_C3,
  ATM_DELAY(32),
  ATM_NOTE_G3,
  ATM_DELAY(16),
  ATM_NOTE_C3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 55" bv_ph  [19b]
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(52),   // -> bassV
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(52),   // -> bassV
  ATM_ADD_TRA(7),
  ATM_GOTO(52),   // -> bassV
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(52),   // -> bassV
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 56" bp_ph  [19b]
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(2),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(2),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 57" bc_ph  [17b]
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(2),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(7),
  ATM_RETURN,

  //"Track 58" bd_ph  [19b]
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(7),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(53),   // -> bassC
  ATM_ADD_TRA(5),
  ATM_RETURN,

  //"Track 59" bb_ph  [19b]
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(54),   // -> bassB
  ATM_ADD_TRA(2),
  ATM_GOTO(54),   // -> bassB
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(54),   // -> bassB
  ATM_ADD_TRA(5),
  ATM_GOTO(54),   // -> bassB
  ATM_ADD_TRA(3),
  ATM_RETURN,

  //"Track 60" b_intro  [13b]
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_VOL(63),
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA(0),
  ATM_GOTO(57),   // -> bc_ph
  ATM_RETURN,

  //"Track 61" b_body  [46b]
  ATM_SET_TRA(0),
  ATM_VOL(63),
  ATM_SL_VOL((uint8_t)-3),
  ATM_REPEAT(1, 55),   // 2x bv_ph
  ATM_GOTO(56),   // -> bp_ph
  ATM_REPEAT(1, 57),   // 2x bc_ph
  ATM_GOTO(58),   // -> bd_ph
  ATM_REPEAT(1, 55),   // 2x bv_ph
  ATM_GOTO(56),   // -> bp_ph
  ATM_REPEAT(1, 57),   // 2x bc_ph
  ATM_GOTO(58),   // -> bd_ph
  ATM_SL_VOL((uint8_t)-1),
  ATM_REPEAT(1, 59),   // 2x bb_ph
  ATM_SL_VOL((uint8_t)-3),
  ATM_REPEAT(1, 55),   // 2x bv_ph
  ATM_GOTO(56),   // -> bp_ph
  ATM_ADD_TRA(2),
  ATM_REPEAT(3, 57),   // 4x bc_ph
  ATM_ADD_TRA((uint8_t)-2),
  ATM_RETURN,

  //"Track 62" k4  [6b]
  ATM_VOL(48),
  ATM_SL_VOL((uint8_t)-12),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 63" s4  [6b]
  ATM_VOL(44),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 64" h4  [6b]
  ATM_VOL(14),
  ATM_SL_VOL((uint8_t)-7),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 65" o4  [6b]
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 66" b1  [6b]
  ATM_GOTO(62),   // -> k4
  ATM_REPEAT(2, 64),   // 3x h4
  ATM_RETURN,

  //"Track 67" b2  [8b]
  ATM_GOTO(63),   // -> s4
  ATM_REPEAT(1, 64),   // 2x h4
  ATM_GOTO(62),   // -> k4
  ATM_RETURN,

  //"Track 68" b3  [9b]
  ATM_GOTO(62),   // -> k4
  ATM_GOTO(64),   // -> h4
  ATM_GOTO(65),   // -> o4
  ATM_GOTO(64),   // -> h4
  ATM_RETURN,

  //"Track 69" b4  [9b]
  ATM_GOTO(63),   // -> s4
  ATM_GOTO(64),   // -> h4
  ATM_GOTO(62),   // -> k4
  ATM_GOTO(64),   // -> h4
  ATM_RETURN,

  //"Track 70" bS  [6b]
  ATM_GOTO(63),   // -> s4
  ATM_REPEAT(2, 64),   // 3x h4
  ATM_RETURN,

  //"Track 71" bhh  [4b]
  ATM_REPEAT(3, 64),   // 4x h4
  ATM_RETURN,

  //"Track 72" bf  [9b]
  ATM_GOTO(62),   // -> k4
  ATM_GOTO(64),   // -> h4
  ATM_GOTO(62),   // -> k4
  ATM_GOTO(64),   // -> h4
  ATM_RETURN,

  //"Track 73" roll16  [10b]
  ATM_VOL(10),
  ATM_SL_VOL(2),
  ATM_TREM(6, 0),
  ATM_DELAY(16),
  ATM_TREM_OFF,
  ATM_RETURN,

  //"Track 74" roll64  [11b]
  ATM_VOL(10),
  ATM_SL_VOL_ADV(1, 1),
  ATM_TREM(6, 0),
  ATM_DELAY(64),
  ATM_TREM_OFF,
  ATM_RETURN,

  //"Track 75" bar_g  [9b]
  ATM_GOTO(66),   // -> b1
  ATM_GOTO(67),   // -> b2
  ATM_GOTO(68),   // -> b3
  ATM_GOTO(69),   // -> b4
  ATM_RETURN,

  //"Track 76" bar_gf  [9b]
  ATM_GOTO(66),   // -> b1
  ATM_GOTO(67),   // -> b2
  ATM_GOTO(68),   // -> b3
  ATM_GOTO(73),   // -> roll16
  ATM_RETURN,

  //"Track 77" bar_v  [9b]
  ATM_GOTO(66),   // -> b1
  ATM_GOTO(70),   // -> bS
  ATM_GOTO(66),   // -> b1
  ATM_GOTO(70),   // -> bS
  ATM_RETURN,

  //"Track 78" bar_vf  [9b]
  ATM_GOTO(66),   // -> b1
  ATM_GOTO(70),   // -> bS
  ATM_GOTO(66),   // -> b1
  ATM_GOTO(73),   // -> roll16
  ATM_RETURN,

  //"Track 79" bar_hat  [4b]
  ATM_REPEAT(3, 71),   // 4x bhh
  ATM_RETURN,

  //"Track 80" bar_df  [4b]
  ATM_REPEAT(3, 72),   // 4x bf
  ATM_RETURN,

  //"Track 81" bar_dfill  [6b]
  ATM_REPEAT(2, 72),   // 3x bf
  ATM_GOTO(73),   // -> roll16
  ATM_RETURN,

  //"Track 82" bar_br  [6b]
  ATM_GOTO(66),   // -> b1
  ATM_REPEAT(2, 71),   // 3x bhh
  ATM_RETURN,

  //"Track 83" dr4v  [6b]
  ATM_REPEAT(2, 77),   // 3x bar_v
  ATM_GOTO(78),   // -> bar_vf
  ATM_RETURN,

  //"Track 84" dr4g  [6b]
  ATM_REPEAT(2, 75),   // 3x bar_g
  ATM_GOTO(76),   // -> bar_gf
  ATM_RETURN,

  //"Track 85" dr4d  [6b]
  ATM_REPEAT(2, 80),   // 3x bar_df
  ATM_GOTO(81),   // -> bar_dfill
  ATM_RETURN,

  //"Track 86" dr_pre  [6b]
  ATM_REPEAT(2, 75),   // 3x bar_g
  ATM_GOTO(74),   // -> roll64
  ATM_RETURN,

  //"Track 87" dr_br  [8b]
  ATM_REPEAT(5, 82),   // 6x bar_br
  ATM_GOTO(75),   // -> bar_g
  ATM_GOTO(74),   // -> roll64
  ATM_RETURN,

  //"Track 88" d_intro  [9b]
  ATM_REPEAT(3, 79),   // 4x bar_hat
  ATM_REPEAT(2, 75),   // 3x bar_g
  ATM_GOTO(76),   // -> bar_gf
  ATM_RETURN,

  //"Track 89" d_body  [31b]
  ATM_REPEAT(1, 83),   // 2x dr4v
  ATM_GOTO(86),   // -> dr_pre
  ATM_REPEAT(1, 84),   // 2x dr4g
  ATM_GOTO(85),   // -> dr4d
  ATM_REPEAT(1, 83),   // 2x dr4v
  ATM_GOTO(86),   // -> dr_pre
  ATM_REPEAT(1, 84),   // 2x dr4g
  ATM_GOTO(85),   // -> dr4d
  ATM_GOTO(87),   // -> dr_br
  ATM_REPEAT(1, 83),   // 2x dr4v
  ATM_GOTO(86),   // -> dr_pre
  ATM_REPEAT(3, 84),   // 4x dr4g
  ATM_RETURN,

};

#endif
