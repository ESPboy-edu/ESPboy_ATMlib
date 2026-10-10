#ifndef SONG_H
#define SONG_H

// ---------------------------------------------------------------------------
//  "Midnight Run" — Synthwave 80s for Arduboy / ATMlib
//
//  Play:  #include "synth.h"    ATM.play(midnightRun);
//
//  Duration : ~3 min 01 s  (96 bars × 64 ticks = 6144 ticks − 1 for restart)
//  Timing   : ATM_SET_TEMPO(17) → ~127.5 BPM
//  Key      : A minor  (Am | F | C | G)
//  Structure:
//    Intro  (bars  1-16) — fade in: bass+drums only, melody enters bar 9
//    A      (bars 17-32) — full band, main synth hook
//    B      (bars 33-48) — bridge: higher melody, faster phrases
//    A'     (bars 49-64) — hook returns
//    C      (bars 65-80) — breakdown: arp pads on CH1, no lead
//    Outro  (bars 81-96) — fade out: melody fades bars 81-88, silent 89-96
//  Channels:
//    CH0 PULSE  — lead synth melody
//    CH1 SQUARE — kick every beat + chord stabs (arp pads in section C)
//    CH2 SAW    — ducked bass (Am/F/C/G via transpose)
//    CH3 NOISE  — kick click, clap 2&4, hats, snare-roll fills every 4 bars
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif


Song midnightRun[] = {     // total song bytes = 715
  0x2F,                       // Number of tracks (47)
  0x00, 0x00,                 // Track 0  @    0  (silent)
  0x03, 0x00,                 // Track 1  @    3  (mel_Am_A)
  0x0E, 0x00,                 // Track 2  @   14  (mel_F_A)
  0x19, 0x00,                 // Track 3  @   25  (mel_C_A)
  0x24, 0x00,                 // Track 4  @   36  (mel_G_A)
  0x2F, 0x00,                 // Track 5  @   47  (mel_secA_4bar)
  0x38, 0x00,                 // Track 6  @   56  (mel_Am_B)
  0x45, 0x00,                 // Track 7  @   69  (mel_F_B)
  0x52, 0x00,                 // Track 8  @   82  (mel_C_B)
  0x5F, 0x00,                 // Track 9  @   95  (mel_G_B)
  0x6C, 0x00,                 // Track 10 @  108  (mel_secB_4bar)
  0x75, 0x00,                 // Track 11 @  117  (silence_8bar)
  0x7F, 0x00,                 // Track 12 @  127  (silence_16bar)
  0x83, 0x00,                 // Track 13 @  131  (silence_8bar_last)
  0x8D, 0x00,                 // Track 14 @  141  (ch0_main)
  0xBC, 0x00,                 // Track 15 @  188  (kick)
  0xC7, 0x00,                 // Track 16 @  199  (beat_Am)
  0xD3, 0x00,                 // Track 17 @  211  (beat_F)
  0xDF, 0x00,                 // Track 18 @  223  (beat_C)
  0xEB, 0x00,                 // Track 19 @  235  (beat_G)
  0xF7, 0x00,                 // Track 20 @  247  (beat_Am_last)
  0x03, 0x01,                 // Track 21 @  259  (chords_4bar)
  0x10, 0x01,                 // Track 22 @  272  (delay64)
  0x12, 0x01,                 // Track 23 @  274  (pad_Am)
  0x1F, 0x01,                 // Track 24 @  287  (pad_F)
  0x2C, 0x01,                 // Track 25 @  300  (pad_C)
  0x39, 0x01,                 // Track 26 @  313  (pad_G)
  0x46, 0x01,                 // Track 27 @  326  (ch1_main)
  0x6C, 0x01,                 // Track 28 @  364  (bass_beat)
  0x78, 0x01,                 // Track 29 @  376  (bass_beat4)
  0x84, 0x01,                 // Track 30 @  388  (bass_beat4_last)
  0x90, 0x01,                 // Track 31 @  400  (bass_bar)
  0x96, 0x01,                 // Track 32 @  406  (bass_bar_last)
  0x9C, 0x01,                 // Track 33 @  412  (bass_4bar)
  0xAD, 0x01,                 // Track 34 @  429  (bass_4bar_last)
  0xBE, 0x01,                 // Track 35 @  446  (bass_main)
  0xC6, 0x01,                 // Track 36 @  454  (drum_A)
  0xD6, 0x01,                 // Track 37 @  470  (drum_B)
  0xE1, 0x01,                 // Track 38 @  481  (drum_pair)
  0xE6, 0x01,                 // Track 39 @  486  (drum_bar)
  0xEA, 0x01,                 // Track 40 @  490  (drum_fill)
  0x13, 0x02,                 // Track 41 @  531  (drum_fill_last)
  0x3C, 0x02,                 // Track 42 @  572  (drum_bar_fill)
  0x43, 0x02,                 // Track 43 @  579  (drum_bar_fill_last)
  0x4A, 0x02,                 // Track 44 @  586  (drum_8bar)
  0x55, 0x02,                 // Track 45 @  597  (drum_8bar_last)
  0x60, 0x02,                 // Track 46 @  608  (drums_main)

  0x0E,                         // CH0 entry → track 14 (ch0_main)
  0x1B,                         // CH1 entry → track 27 (ch1_main)
  0x23,                         // CH2 entry → track 35 (bass_main)
  0x2E,                         // CH3 entry → track 46 (drums_main)

  // Track  0: silent  [3b]  dummy track 0
  ATM_VOL(0),
  ATM_STOP_CHAN,

  // Track  1: mel_Am_A  [11b]  A-section Am bar
  ATM_NOTE_E5,
  ATM_DELAY(16),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(16),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  2: mel_F_A  [11b]  A-section F bar
  ATM_NOTE_F5,
  ATM_DELAY(16),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_F5,
  ATM_DELAY(16),
  ATM_NOTE_C5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  3: mel_C_A  [11b]  A-section C bar
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_G5,
  ATM_DELAY(12),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  4: mel_G_A  [11b]  A-section G bar
  ATM_NOTE_D5,
  ATM_DELAY(16),
  ATM_NOTE_B4,
  ATM_DELAY(8),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  5: mel_secA_4bar  [9b]  melody A: 4 bars Am F C G
  ATM_GOTO(1),
  ATM_GOTO(2),
  ATM_GOTO(3),
  ATM_GOTO(4),
  ATM_RETURN,

  // Track  6: mel_Am_B  [13b]  B-section Am bar (higher, faster)
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_E5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  7: mel_F_B  [13b]  B-section F bar
  ATM_NOTE_A5,
  ATM_DELAY(8),
  ATM_NOTE_F5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_F5,
  ATM_DELAY(16),
  ATM_NOTE_C5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  8: mel_C_B  [13b]  B-section C bar
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(16),
  ATM_NOTE_C5,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track  9: mel_G_B  [13b]  B-section G bar
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_B4,
  ATM_DELAY(8),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(16),
  ATM_NOTE_G4,
  ATM_DELAY(16),
  ATM_RETURN,

  // Track 10: mel_secB_4bar  [9b]  melody B: 4 bars Am F C G
  ATM_GOTO(6),
  ATM_GOTO(7),
  ATM_GOTO(8),
  ATM_GOTO(9),
  ATM_RETURN,

  // Track 11: silence_8bar  [10b]  8 bars of silence (512t)
  0x00, /* note off/silence */
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_RETURN,

  // Track 12: silence_16bar  [4b]  16 bars silence (1024t)
  ATM_REPEAT(1,11),
  ATM_RETURN,

  // Track 13: silence_8bar_last  [10b]  7 bars + 63t silence (511t total)
  0x00, /* note off/silence */
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(64),
  ATM_DELAY(63),
  ATM_RETURN,

  // Track 14: ch0_main  [47b]  CH0 PULSE: full 3-minute song
  ATM_VOL(0),
  ATM_GOTO(11),
  ATM_VOL(18),
  ATM_REPEAT(1,5),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(3,5),
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(3,10),
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_REPEAT(3,5),
  ATM_VOL(0),
  ATM_GOTO(12),
  ATM_VOL(28),
  ATM_GOTO(5),
  ATM_VOL(18),
  ATM_GOTO(5),
  ATM_VOL(0),
  ATM_GOTO(13),
  ATM_STOP_CHAN,

  // Track 15: kick  [11b]  pitch-drop kick (8t)
  ATM_ARP_OFF,
  ATM_SL_FRQ((uint8_t)-127),
  ATM_VOL(30),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_RETURN,

  // Track 16: beat_Am  [12b]  beat: kick + Am stab
  ATM_GOTO(15),
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x37,0x20),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  // Track 17: beat_F  [12b]  beat: kick + F stab
  ATM_GOTO(15),
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x47,0x20),
  ATM_NOTE_F3,
  ATM_DELAY(8),
  ATM_RETURN,

  // Track 18: beat_C  [12b]  beat: kick + C stab
  ATM_GOTO(15),
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x47,0x20),
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_RETURN,

  // Track 19: beat_G  [12b]  beat: kick + G stab
  ATM_GOTO(15),
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x47,0x20),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  // Track 20: beat_Am_last  [12b]  last beat (63t stab loop comp)
  ATM_GOTO(15),
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x37,0x20),
  ATM_NOTE_A3,
  ATM_DELAY(7),
  ATM_RETURN,

  // Track 21: chords_4bar  [13b]  CH1: 4-bar Am F C G chord group
  ATM_REPEAT(3,16),
  ATM_REPEAT(3,17),
  ATM_REPEAT(3,18),
  ATM_REPEAT(3,19),
  ATM_RETURN,

  // Track 22: delay64  [2b]  one 64-tick delay unit
  ATM_DELAY(64),
  ATM_RETURN,

  // Track 23: pad_Am  [13b]  C-section Am arp pad (256t)
  ATM_ARP_OFF,
  ATM_SL_VOL((uint8_t)-1),
  ATM_VOL(20),
  ATM_ARP(0x37,0x02),
  ATM_NOTE_A3,
  ATM_REPEAT(3,22),
  ATM_RETURN,

  // Track 24: pad_F  [13b]  C-section F arp pad (256t)
  ATM_ARP_OFF,
  ATM_SL_VOL((uint8_t)-1),
  ATM_VOL(20),
  ATM_ARP(0x47,0x02),
  ATM_NOTE_F3,
  ATM_REPEAT(3,22),
  ATM_RETURN,

  // Track 25: pad_C  [13b]  C-section C arp pad (256t)
  ATM_ARP_OFF,
  ATM_SL_VOL((uint8_t)-1),
  ATM_VOL(20),
  ATM_ARP(0x47,0x02),
  ATM_NOTE_C3,
  ATM_REPEAT(3,22),
  ATM_RETURN,

  // Track 26: pad_G  [13b]  C-section G arp pad (256t)
  ATM_ARP_OFF,
  ATM_SL_VOL((uint8_t)-1),
  ATM_VOL(20),
  ATM_ARP(0x47,0x02),
  ATM_NOTE_G3,
  ATM_REPEAT(3,22),
  ATM_RETURN,

  // Track 27: ch1_main  [38b]  CH1 SQUARE: full song timeline
  ATM_REPEAT(3,21),
  ATM_REPEAT(3,21),
  ATM_REPEAT(3,21),
  ATM_REPEAT(3,21),
  ATM_GOTO(23),
  ATM_GOTO(24),
  ATM_GOTO(25),
  ATM_GOTO(26),
  ATM_REPEAT(2,21),
  ATM_REPEAT(3,16),
  ATM_REPEAT(3,17),
  ATM_REPEAT(3,18),
  ATM_REPEAT(2,19),
  ATM_GOTO(20),
  ATM_STOP_CHAN,

  // Track 28: bass_beat  [12b]  bass beat: silence-root-oct-root (16t)
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(55),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_RETURN,

  // Track 29: bass_beat4  [12b]  bass beat4: silence-root-root-oct (16t)
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(55),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_RETURN,

  // Track 30: bass_beat4_last  [12b]  bass last beat4: 15t (loop comp)
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(55),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(3),
  ATM_RETURN,

  // Track 31: bass_bar  [6b]  bass bar (64t)
  ATM_REPEAT(2,28),
  ATM_GOTO(29),
  ATM_RETURN,

  // Track 32: bass_bar_last  [6b]  last bass bar (63t)
  ATM_REPEAT(2,28),
  ATM_GOTO(30),
  ATM_RETURN,

  // Track 33: bass_4bar  [17b]  bass 4-bar: Am F C G
  ATM_SET_TRA(0),
  ATM_GOTO(31),
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(31),
  ATM_SET_TRA(3),
  ATM_GOTO(31),
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(31),
  ATM_RETURN,

  // Track 34: bass_4bar_last  [17b]  last 4-bar group (bar 96 is 63t)
  ATM_SET_TRA(0),
  ATM_GOTO(31),
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(31),
  ATM_SET_TRA(3),
  ATM_GOTO(31),
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(32),
  ATM_RETURN,

  // Track 35: bass_main  [8b]  CH2 SAW: 96 bars (last bar 63t)
  ATM_SL_VOL((uint8_t)-5),
  ATM_REPEAT(22,33),
  ATM_GOTO(34),
  ATM_STOP_CHAN,

  // Track 36: drum_A  [16b]  beat 1/3: click hat (16t)
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

  // Track 37: drum_B  [11b]  beat 2/4: clap + hat (16t)
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(8),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(8),
  ATM_RETURN,

  // Track 38: drum_pair  [5b]  beats 1+2 (32t)
  ATM_GOTO(36),
  ATM_GOTO(37),
  ATM_RETURN,

  // Track 39: drum_bar  [4b]  one bar (64t)
  ATM_REPEAT(1,38),
  ATM_RETURN,

  // Track 40: drum_fill  [41b]  snare roll 16t
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

  // Track 41: drum_fill_last  [41b]  snare roll 15t (loop comp)
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
  ATM_DELAY(1),
  ATM_RETURN,

  // Track 42: drum_bar_fill  [7b]  bar 4: pair+drumA+fill
  ATM_GOTO(38),
  ATM_GOTO(36),
  ATM_GOTO(40),
  ATM_RETURN,

  // Track 43: drum_bar_fill_last  [7b]  bar 8: pair+drumA+fill_last
  ATM_GOTO(38),
  ATM_GOTO(36),
  ATM_GOTO(41),
  ATM_RETURN,

  // Track 44: drum_8bar  [11b]  8-bar drum section (with fill)
  ATM_REPEAT(2,39),
  ATM_GOTO(42),
  ATM_REPEAT(2,39),
  ATM_GOTO(42),
  ATM_RETURN,

  // Track 45: drum_8bar_last  [11b]  last 8-bar drum section (fill_last at end)
  ATM_REPEAT(2,39),
  ATM_GOTO(42),
  ATM_REPEAT(2,39),
  ATM_GOTO(43),
  ATM_RETURN,

  // Track 46: drums_main  [8b]  CH3 NOISE: 96 bars (no loop, 1-shot)
  ATM_SET_TEMPO(40),
  ATM_REPEAT(10,44),
  ATM_GOTO(45),
  ATM_STOP_CHAN,

};

#endif
