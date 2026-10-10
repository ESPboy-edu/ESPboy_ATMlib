#ifndef SONG_H
#define SONG_H

// ---------------------------------------------------------------------------
//  "Cutlass Cove" - an original tropical-pirate showcase for ATMlib       839 bytes
//
//  Play with:   #include "song.h"
//               ATM.play(cutlassCove);
//
//  A ONE-SHOT: it ends by itself after 2048 ticks (32 bars).  Tempo is set INSIDE the song
//  (ATM_SET_TEMPO(30), then ATM_ADD_TEMPO changes), so do not call ATM.setTempo() while it plays.
//
//  FORM (1 bar = 64 ticks = 4 beats of 16 ticks)
//     bars  1- 4  INTRO   fade-in: sea, seagulls, arpeggio pad, bass swell        CUE 1
//     bars  5-12  A       C | F | G | C   Am | F | G | C   (calypso / reggae)       CUE 2
//     bars 13-20  B       Am G F E   Dm Am F G7  (storm, tempo +6)                CUE 3
//     bars 21-28  A'      same as A, lifted a whole step: D | G | A | D ...       CUE 4
//     bars 29-32  OUTRO   ritardando (tempo -1 per beat) + fade-out               CUE 5
//     last tick                                                                   CUE 6
//  Poll the cues from your sketch with  uint8_t c = ATM.check();  (0 = nothing new)
//
//  WHAT IT SHOWS OFF  (nearly every ATMlib command)
//     ATM_SET_TEMPO / ATM_ADD_TEMPO      A->B speed-up, B->A' reset, outro ritardando (-1 per beat)
//     ATM_CUE                            section markers 1..6 (and a final "song ended" 6)
//     ATM_GOTO / ATM_REPEAT / ATM_RETURN nested tracks: bars are tracks, phrases call bars, depth 4
//     ATM_ADD_TRA / ATM_SET_TRA          ONE riff per chord on lead + bass (C F G C, Am F G C ...),
//                                        the whole A' section is the A tracks + SET_TRA(2)
//     ATM_ARP                            CH1 pad (slow, 4 ticks/step), skank + stabs (1 tick/step),
//                                        major 0x43, minor 0x34 and a G7 shell 0x46
//     ATM_VOL / ATM_SL_VOL               plucked steel-drum lead, skank decay, drum hits
//     ATM_SL_VOL_ADV / ATM_SL_VOL_OFF    long fades: +1 every 5-9 ticks in, -1 every 4-7 ticks out
//     ATM_SL_FRQ                         the cannon "boom" on the bass at the start of B
//     ATM_GLIS                           the two seagull cries (falling glissando)
//     ATM_VIB                            pan-flute / singing lead
//     ATM_CUT                            staccato pedal riff in B (gate: 4 ticks on, 4 off)
//     ATM_TREM                           sea swell (slow) and the snare roll (flip every tick)
//     ATM_NOISE                          thunder rumble at the start of B
//     ATM_STOP_CHAN                      every channel stops by itself at the end
//  Not used here: ATM_GOTO_ADV (loops), ATM_SL_FRQ_ADV, the *_OFF of arp/transpose.
//
//  CHANNELS (ATMlib defaults)
//     CH0 PULSE   lead      CH1 SQUARE  chords      CH2 SAW  bass      CH3 NOISE  drums + sea + cues
//
//  Needs every ATM_FX_* option left ON (the default).  Track 0 is only CH3's entry point and is
//  never called: ATMlib starts every channel with "current track = 0".
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif



Song cutlassCove[] = {     // total song bytes = 839
  0x33,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0      0   drums
  0x41, 0x00,                 // Address of track 1     65   hat
  0x47, 0x00,                 // Address of track 2     71   ohat
  0x4D, 0x00,                 // Address of track 3     77   thump
  0x53, 0x00,                 // Address of track 4     83   kick
  0x59, 0x00,                 // Address of track 5     89   snr
  0x5F, 0x00,                 // Address of track 6     95   roll
  0x69, 0x00,                 // Address of track 7    105   thunder
  0x72, 0x00,                 // Address of track 8    114   bar_a
  0x7D, 0x00,                 // Address of track 9    125   fill_a
  0x87, 0x00,                 // Address of track 10   135   dr_a4
  0x8D, 0x00,                 // Address of track 11   141   half_b
  0x96, 0x00,                 // Address of track 12   150   bar_b
  0x9A, 0x00,                 // Address of track 13   154   fill_b
  0xA3, 0x00,                 // Address of track 14   163   dr_b4
  0xA9, 0x00,                 // Address of track 15   169   slow
  0xAD, 0x00,                 // Address of track 16   173   lead
  0x09, 0x01,                 // Address of track 17   265   gull
  0x1A, 0x01,                 // Address of track 18   282   sing
  0x21, 0x01,                 // Address of track 19   289   riff_a
  0x32, 0x01,                 // Address of track 20   306   riff_b
  0x41, 0x01,                 // Address of track 21   321   phr_a
  0x50, 0x01,                 // Address of track 22   336   riff_p
  0x61, 0x01,                 // Address of track 23   353   chords
  0xA0, 0x01,                 // Address of track 24   416   sk_C
  0xA6, 0x01,                 // Address of track 25   422   sk_F
  0xAC, 0x01,                 // Address of track 26   428   sk_G
  0xB2, 0x01,                 // Address of track 27   434   sk_Am
  0xB8, 0x01,                 // Address of track 28   440   sk_D
  0xBE, 0x01,                 // Address of track 29   446   sk_A
  0xC4, 0x01,                 // Address of track 30   452   sk_Bm
  0xCA, 0x01,                 // Address of track 31   458   sk_a1
  0xD3, 0x01,                 // Address of track 32   467   sk_a2
  0xE2, 0x01,                 // Address of track 33   482   sk_a1b
  0xEB, 0x01,                 // Address of track 34   491   sk_a2b
  0xFA, 0x01,                 // Address of track 35   506   st_Am
  0xFD, 0x01,                 // Address of track 36   509   st_G
  0x00, 0x02,                 // Address of track 37   512   st_F
  0x03, 0x02,                 // Address of track 38   515   st_E
  0x06, 0x02,                 // Address of track 39   518   st_Dm
  0x09, 0x02,                 // Address of track 40   521   st_b1
  0x1C, 0x02,                 // Address of track 41   540   st_b2
  0x32, 0x02,                 // Address of track 42   562   bass
  0x87, 0x02,                 // Address of track 43   647   bass_h
  0x90, 0x02,                 // Address of track 44   656   bass_a
  0x94, 0x02,                 // Address of track 45   660   bass_f
  0x9F, 0x02,                 // Address of track 46   671   bass_p1
  0xAE, 0x02,                 // Address of track 47   686   bass_p2
  0xBF, 0x02,                 // Address of track 48   703   bass_bx
  0xCC, 0x02,                 // Address of track 49   716   bass_b
  0xD3, 0x02,                 // Address of track 50   723   boom

  0x10,                         // CH0 entry -> track 16 (lead)
  0x17,                         // CH1 entry -> track 23 (chords)
  0x2A,                         // CH2 entry -> track 42 (bass)
  0x00,                         // CH3 entry -> track 0 (drums)

  //"Track 0" drums  [65b]
  ATM_SET_TEMPO(30),
  ATM_CUE(1),
  ATM_VOL(4),
  ATM_SL_VOL_ADV(1, 7),
  ATM_TREM(1, 15),
  ATM_DELAY(64),
  ATM_DELAY(32),
  ATM_SL_VOL_OFF,
  ATM_DELAY(32),
  ATM_DELAY(64),
  ATM_DELAY(48),
  ATM_GOTO(6),   // -> roll
  ATM_CUE(2),
  ATM_REPEAT(1, 10),   // 2x dr_a4
  ATM_CUE(3),
  ATM_ADD_TEMPO(6),
  ATM_GOTO(7),   // -> thunder
  ATM_GOTO(11),   // -> half_b
  ATM_REPEAT(1, 12),   // 2x bar_b
  ATM_GOTO(13),   // -> fill_b
  ATM_GOTO(14),   // -> dr_b4
  ATM_CUE(4),
  ATM_SET_TEMPO(30),
  ATM_REPEAT(1, 10),   // 2x dr_a4
  ATM_CUE(5),
  ATM_GOTO(8),   // -> bar_a
  ATM_VOL(20),
  ATM_SL_VOL_ADV((uint8_t)-1, 5),
  ATM_TREM(1, 15),
  ATM_REPEAT(11, 15),   // 12x slow
  ATM_CUE(6),
  ATM_STOP_CHAN,

  //"Track 1" hat  [6b]
  ATM_VOL(14),
  ATM_SL_VOL((uint8_t)-7),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" ohat  [6b]
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 3" thump  [6b]
  ATM_VOL(46),
  ATM_SL_VOL((uint8_t)-6),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" kick  [6b]
  ATM_VOL(48),
  ATM_SL_VOL((uint8_t)-12),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 5" snr  [6b]
  ATM_VOL(44),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 6" roll  [10b]
  ATM_VOL(10),
  ATM_SL_VOL(2),
  ATM_TREM(6, 0),
  ATM_DELAY(16),
  ATM_TREM_OFF,
  ATM_RETURN,

  //"Track 7" thunder  [9b]
  ATM_NOISE(0x14),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_DELAY(32),
  ATM_NOISE_OFF,
  ATM_RETURN,

  //"Track 8" bar_a  [11b]
  ATM_REPEAT(3, 1),   // 4x hat
  ATM_GOTO(3),   // -> thump
  ATM_REPEAT(1, 1),   // 2x hat
  ATM_GOTO(2),   // -> ohat
  ATM_RETURN,

  //"Track 9" fill_a  [10b]
  ATM_REPEAT(3, 1),   // 4x hat
  ATM_GOTO(3),   // -> thump
  ATM_GOTO(1),   // -> hat
  ATM_GOTO(6),   // -> roll
  ATM_RETURN,

  //"Track 10" dr_a4  [6b]
  ATM_REPEAT(2, 8),   // 3x bar_a
  ATM_GOTO(9),   // -> fill_a
  ATM_RETURN,

  //"Track 11" half_b  [9b]
  ATM_GOTO(4),   // -> kick
  ATM_GOTO(1),   // -> hat
  ATM_GOTO(5),   // -> snr
  ATM_GOTO(1),   // -> hat
  ATM_RETURN,

  //"Track 12" bar_b  [4b]
  ATM_REPEAT(1, 11),   // 2x half_b
  ATM_RETURN,

  //"Track 13" fill_b  [9b]
  ATM_GOTO(11),   // -> half_b
  ATM_GOTO(4),   // -> kick
  ATM_GOTO(1),   // -> hat
  ATM_GOTO(6),   // -> roll
  ATM_RETURN,

  //"Track 14" dr_b4  [6b]
  ATM_REPEAT(2, 12),   // 3x bar_b
  ATM_GOTO(13),   // -> fill_b
  ATM_RETURN,

  //"Track 15" slow  [4b]
  ATM_ADD_TEMPO((uint8_t)-1),
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 16" lead  [92b]
  ATM_DELAY(64),
  ATM_GOTO(17),   // -> gull
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-1),
  ATM_VIB(3, 3),
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(18),   // -> sing
  ATM_ADD_TRA(2),
  ATM_GOTO(18),   // -> sing
  ATM_VIB_OFF,
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_REPEAT(1, 21),   // 2x phr_a
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(0),
  ATM_CUT(0x23),
  ATM_REPEAT(3, 22),   // 4x riff_p
  ATM_CUT_OFF,
  ATM_VIB(3, 3),
  ATM_SET_TRA(5),
  ATM_GOTO(18),   // -> sing
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(18),   // -> sing
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(18),   // -> sing
  ATM_ADD_TRA(2),
  ATM_GOTO(18),   // -> sing
  ATM_VIB_OFF,
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-1),
  ATM_SET_TRA(2),
  ATM_REPEAT(1, 21),   // 2x phr_a
  ATM_VOL(34),
  ATM_SL_VOL_ADV((uint8_t)-1, 5),
  ATM_VIB(3, 3),
  ATM_SET_TRA(5),
  ATM_GOTO(18),   // -> sing
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(18),   // -> sing
  ATM_ADD_TRA(7),
  ATM_GOTO(18),   // -> sing
  ATM_NOTE_E5,
  ATM_DELAY(64),
  ATM_STOP_CHAN,

  //"Track 17" gull  [17b]
  ATM_VOL(24),
  ATM_SL_VOL((uint8_t)-1),
  ATM_NOTE_G6,
  ATM_GLIS(0x82),
  ATM_DELAY(18),
  ATM_GLIS_OFF,
  ATM_DELAY(14),
  ATM_NOTE_E6,
  ATM_GLIS(0x82),
  ATM_DELAY(12),
  ATM_GLIS_OFF,
  ATM_DELAY(20),
  ATM_RETURN,

  //"Track 18" sing  [7b]
  ATM_NOTE_E5,
  ATM_DELAY(32),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_B5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 19" riff_a  [17b]
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_G5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_D5,
  ATM_DELAY(4),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 20" riff_b  [15b]
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_G5,
  ATM_DELAY(4),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 21" phr_a  [15b]
  ATM_GOTO(19),   // -> riff_a
  ATM_ADD_TRA(5),
  ATM_GOTO(19),   // -> riff_a
  ATM_ADD_TRA(2),
  ATM_GOTO(19),   // -> riff_a
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(20),   // -> riff_b
  ATM_RETURN,

  //"Track 22" riff_p  [17b]
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 23" chords  [63b]
  ATM_VOL(0),
  ATM_SL_VOL_ADV(1, 8),
  ATM_ARP(0x43, 0x23),
  ATM_NOTE_C4,
  ATM_DELAY(64),
  ATM_NOTE_C4,
  ATM_DELAY(64),
  ATM_NOTE_F4,
  ATM_DELAY(64),
  ATM_NOTE_G4,
  ATM_DELAY(64),
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-6),
  ATM_ARP(0x43, 0x20),
  ATM_GOTO(31),   // -> sk_a1
  ATM_GOTO(32),   // -> sk_a2
  ATM_VOL(28),
  ATM_SL_VOL((uint8_t)-1),
  ATM_GOTO(40),   // -> st_b1
  ATM_GOTO(41),   // -> st_b2
  ATM_VOL(30),
  ATM_SL_VOL((uint8_t)-6),
  ATM_ARP(0x43, 0x20),
  ATM_GOTO(33),   // -> sk_a1b
  ATM_GOTO(34),   // -> sk_a2b
  ATM_VOL(26),
  ATM_SL_VOL_ADV((uint8_t)-1, 6),
  ATM_ARP(0x43, 0x23),
  ATM_NOTE_D4,
  ATM_DELAY(64),
  ATM_NOTE_G4,
  ATM_DELAY(64),
  ATM_NOTE_D4,
  ATM_DELAY(64),
  ATM_NOTE_A4,
  ATM_DELAY(64),
  ATM_STOP_CHAN,

  //"Track 24" sk_C  [6b]
  ATM_DELAY(16),
  ATM_NOTE_C4,
  ATM_DELAY(32),
  ATM_NOTE_C4,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 25" sk_F  [6b]
  ATM_DELAY(16),
  ATM_NOTE_F4,
  ATM_DELAY(32),
  ATM_NOTE_F4,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 26" sk_G  [6b]
  ATM_DELAY(16),
  ATM_NOTE_G4,
  ATM_DELAY(32),
  ATM_NOTE_G4,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 27" sk_Am  [6b]
  ATM_DELAY(16),
  ATM_NOTE_A3,
  ATM_DELAY(32),
  ATM_NOTE_A3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 28" sk_D  [6b]
  ATM_DELAY(16),
  ATM_NOTE_D4,
  ATM_DELAY(32),
  ATM_NOTE_D4,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 29" sk_A  [6b]
  ATM_DELAY(16),
  ATM_NOTE_A4,
  ATM_DELAY(32),
  ATM_NOTE_A4,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 30" sk_Bm  [6b]
  ATM_DELAY(16),
  ATM_NOTE_B3,
  ATM_DELAY(32),
  ATM_NOTE_B3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 31" sk_a1  [9b]
  ATM_GOTO(24),   // -> sk_C
  ATM_GOTO(25),   // -> sk_F
  ATM_GOTO(26),   // -> sk_G
  ATM_GOTO(24),   // -> sk_C
  ATM_RETURN,

  //"Track 32" sk_a2  [15b]
  ATM_ARP(0x34, 0x20),
  ATM_GOTO(27),   // -> sk_Am
  ATM_ARP(0x43, 0x20),
  ATM_GOTO(25),   // -> sk_F
  ATM_GOTO(26),   // -> sk_G
  ATM_GOTO(24),   // -> sk_C
  ATM_RETURN,

  //"Track 33" sk_a1b  [9b]
  ATM_GOTO(28),   // -> sk_D
  ATM_GOTO(26),   // -> sk_G
  ATM_GOTO(29),   // -> sk_A
  ATM_GOTO(28),   // -> sk_D
  ATM_RETURN,

  //"Track 34" sk_a2b  [15b]
  ATM_ARP(0x34, 0x20),
  ATM_GOTO(30),   // -> sk_Bm
  ATM_ARP(0x43, 0x20),
  ATM_GOTO(26),   // -> sk_G
  ATM_GOTO(29),   // -> sk_A
  ATM_GOTO(28),   // -> sk_D
  ATM_RETURN,

  //"Track 35" st_Am  [3b]
  ATM_NOTE_A3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 36" st_G  [3b]
  ATM_NOTE_G3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 37" st_F  [3b]
  ATM_NOTE_F3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 38" st_E  [3b]
  ATM_NOTE_E3,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 39" st_Dm  [3b]
  ATM_NOTE_D4,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 40" st_b1  [19b]
  ATM_ARP(0x34, 0x20),
  ATM_REPEAT(3, 35),   // 4x st_Am
  ATM_ARP(0x43, 0x20),
  ATM_REPEAT(3, 36),   // 4x st_G
  ATM_REPEAT(3, 37),   // 4x st_F
  ATM_REPEAT(3, 38),   // 4x st_E
  ATM_RETURN,

  //"Track 41" st_b2  [22b]
  ATM_ARP(0x34, 0x20),
  ATM_REPEAT(3, 39),   // 4x st_Dm
  ATM_REPEAT(3, 35),   // 4x st_Am
  ATM_ARP(0x43, 0x20),
  ATM_REPEAT(3, 37),   // 4x st_F
  ATM_ARP(0x46, 0x20),
  ATM_REPEAT(3, 36),   // 4x st_G
  ATM_RETURN,

  //"Track 42" bass  [85b]
  ATM_VOL(0),
  ATM_SL_VOL_ADV(1, 4),
  ATM_NOTE_C3,
  ATM_DELAY(64),
  ATM_NOTE_C3,
  ATM_DELAY(64),
  ATM_NOTE_F3,
  ATM_DELAY(64),
  ATM_NOTE_G3,
  ATM_DELAY(64),
  ATM_VOL(63),
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(46),   // -> bass_p1
  ATM_GOTO(47),   // -> bass_p2
  ATM_SL_VOL((uint8_t)-3),
  ATM_SET_TRA((uint8_t)-3),
  ATM_GOTO(50),   // -> boom
  ATM_GOTO(48),   // -> bass_bx
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(49),   // -> bass_b
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(49),   // -> bass_b
  ATM_ADD_TRA((uint8_t)-1),
  ATM_GOTO(49),   // -> bass_b
  ATM_ADD_TRA(10),
  ATM_GOTO(49),   // -> bass_b
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(49),   // -> bass_b
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(49),   // -> bass_b
  ATM_ADD_TRA(2),
  ATM_GOTO(49),   // -> bass_b
  ATM_VOL(63),
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(2),
  ATM_GOTO(46),   // -> bass_p1
  ATM_GOTO(47),   // -> bass_p2
  ATM_VOL(63),
  ATM_SL_VOL_ADV((uint8_t)-1, 3),
  ATM_SET_TRA(0),
  ATM_NOTE_D3,
  ATM_DELAY(64),
  ATM_NOTE_G2,
  ATM_DELAY(64),
  ATM_NOTE_D3,
  ATM_DELAY(64),
  ATM_NOTE_D2,
  ATM_DELAY(64),
  ATM_STOP_CHAN,

  //"Track 43" bass_h  [9b]
  ATM_NOTE_C3,
  ATM_DELAY(12),
  ATM_NOTE_G3,
  ATM_DELAY(4),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 44" bass_a  [4b]
  ATM_REPEAT(1, 43),   // 2x bass_h
  ATM_RETURN,

  //"Track 45" bass_f  [11b]
  ATM_GOTO(43),   // -> bass_h
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_D3,
  ATM_DELAY(8),
  ATM_NOTE_E3,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 46" bass_p1  [15b]
  ATM_GOTO(44),   // -> bass_a
  ATM_ADD_TRA(5),
  ATM_GOTO(44),   // -> bass_a
  ATM_ADD_TRA(2),
  ATM_GOTO(44),   // -> bass_a
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(45),   // -> bass_f
  ATM_RETURN,

  //"Track 47" bass_p2  [17b]
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(44),   // -> bass_a
  ATM_ADD_TRA(8),
  ATM_GOTO(44),   // -> bass_a
  ATM_ADD_TRA(2),
  ATM_GOTO(44),   // -> bass_a
  ATM_ADD_TRA((uint8_t)-7),
  ATM_GOTO(45),   // -> bass_f
  ATM_RETURN,

  //"Track 48" bass_bx  [13b]
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 49" bass_b  [7b]
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_GOTO(48),   // -> bass_bx
  ATM_RETURN,

  //"Track 50" boom  [9b]
  ATM_VOL(63),
  ATM_SL_FRQ((uint8_t)-24),
  ATM_NOTE_C3,
  ATM_DELAY(16),
  ATM_SL_VOL((uint8_t)-3),
  ATM_RETURN,

};

#endif
