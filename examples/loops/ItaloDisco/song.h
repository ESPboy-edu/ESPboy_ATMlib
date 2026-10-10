#ifndef SONG_H
#define SONG_H

#define Song const uint8_t PROGMEM

// ---------------------------------------------------------------------------
//  "Italo Night" - 8 bars, A minor, ~127.5 BPM, ~15.06 s, loops seamlessly
//
//  Play with:   #include "newsong.h"      ATM.play(italoNight);
//
//  Timing : ATM_SET_TEMPO(17) -> ~34 ticks/s.  1 beat = 16 ticks, 1 bar = 64 ticks
//           8 bars = 512 ticks; every channel sums to exactly 512 ticks of delay;
//           each channel's entry track ends with ATM_GOTO(itself), so the loop
//           restarts in the same tick: no STOP, no silent tick, no hop.
//  Chords : Am | F | C | G | Am | F | G | E
//  Channels (ATMlib defaults):
//     CH0 PULSE  - lead melody
//     CH1 SQUARE - pitch-dropping kick on every beat + offbeat arpeggio chord stabs
//     CH2 SAW    - ducked octave bass (silent on the kick, then root-octave-root)
//     CH3 NOISE  - kick click, clap on 2 & 4, closed/open hats, snare roll fills (bars 4 and 8)
//  Pitch  : ATMlib's note table sounds about an octave above the note names and
//           ~80 cents flat (ATM_NOTE_A2 is ~210 Hz). Same for every ATMlib song.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif


Song italoNight[] = {     // total song bytes = 469
  0x1C,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0      0   silent
  0x03, 0x00,                 // Address of track 1      3   lead
  0x19, 0x00,                 // Address of track 2     25   lead_bar1
  0x26, 0x00,                 // Address of track 3     38   lead_bar2
  0x33, 0x00,                 // Address of track 4     51   lead_bar3
  0x40, 0x00,                 // Address of track 5     64   lead_bar4
  0x4D, 0x00,                 // Address of track 6     77   lead_bar5
  0x5E, 0x00,                 // Address of track 7     94   lead_bar6
  0x6F, 0x00,                 // Address of track 8    111   lead_bar7
  0x80, 0x00,                 // Address of track 9    128   lead_bar8
  0x91, 0x00,                 // Address of track 10   145   chords
  0xAD, 0x00,                 // Address of track 11   173   kick
  0xB8, 0x00,                 // Address of track 12   184   beat_Am
  0xC4, 0x00,                 // Address of track 13   196   beat_F
  0xD0, 0x00,                 // Address of track 14   208   beat_C
  0xDC, 0x00,                 // Address of track 15   220   beat_G
  0xE8, 0x00,                 // Address of track 16   232   beat_E
  0xF4, 0x00,                 // Address of track 17   244   bass
  0x18, 0x01,                 // Address of track 18   280   bass_beat
  0x24, 0x01,                 // Address of track 19   292   bass_beat4
  0x30, 0x01,                 // Address of track 20   304   bass_bar
  0x36, 0x01,                 // Address of track 21   310   drums
  0x44, 0x01,                 // Address of track 22   324   drum_A
  0x54, 0x01,                 // Address of track 23   340   drum_B
  0x5F, 0x01,                 // Address of track 24   351   drum_pair
  0x64, 0x01,                 // Address of track 25   356   drum_bar
  0x68, 0x01,                 // Address of track 26   360   drum_fill
  0x91, 0x01,                 // Address of track 27   401   drum_bar_fill

  0x01,                         // Channel 0 entry track  (lead)
  0x0A,                         // Channel 1 entry track  (chords)
  0x11,                         // Channel 2 entry track  (bass)
  0x15,                         // Channel 3 entry track  (drums)

  //"Track 0" silent: unused dummy (never called; index 0 is the 'stale track' slot in ATMlib)
  ATM_VOL(0),
  ATM_STOP_CHAN,

  //"Track 1" lead: CH0 PULSE lead: setup + 8 bars
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
  ATM_GOTO(2),   // -> lead_bar1
  ATM_GOTO(3),   // -> lead_bar2
  ATM_GOTO(4),   // -> lead_bar3
  ATM_GOTO(5),   // -> lead_bar4
  ATM_GOTO(6),   // -> lead_bar5
  ATM_GOTO(7),   // -> lead_bar6
  ATM_GOTO(8),   // -> lead_bar7
  ATM_GOTO(9),   // -> lead_bar8
  ATM_GOTO(1),   // loop: restart this track (no STOP, no gap)

  //"Track 2" lead_bar1: lead bar 1 (Am)
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_C5,
  ATM_DELAY(12),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_C5,
  ATM_DELAY(12),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 3" lead_bar2: lead bar 2 (F)
  ATM_NOTE_F5,
  ATM_DELAY(12),
  ATM_NOTE_C5,
  ATM_DELAY(12),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_F5,
  ATM_DELAY(12),
  ATM_NOTE_C5,
  ATM_DELAY(12),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" lead_bar3: lead bar 3 (C)
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_C5,
  ATM_DELAY(12),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(12),
  ATM_NOTE_G5,
  ATM_DELAY(12),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 5" lead_bar4: lead bar 4 (G)
  ATM_NOTE_D5,
  ATM_DELAY(12),
  ATM_NOTE_B4,
  ATM_DELAY(12),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(12),
  ATM_NOTE_B4,
  ATM_DELAY(12),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 6" lead_bar5: lead bar 5 (Am)
  ATM_NOTE_A4,
  ATM_DELAY(4),
  ATM_NOTE_C5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_A5,
  ATM_DELAY(16),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 7" lead_bar6: lead bar 6 (F)
  ATM_NOTE_F4,
  ATM_DELAY(4),
  ATM_NOTE_A4,
  ATM_DELAY(4),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_F5,
  ATM_DELAY(16),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_A4,
  ATM_DELAY(8),
  ATM_NOTE_F4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 8" lead_bar7: lead bar 7 (G)
  ATM_NOTE_G4,
  ATM_DELAY(4),
  ATM_NOTE_B4,
  ATM_DELAY(4),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(16),
  ATM_NOTE_F5,
  ATM_DELAY(8),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_B4,
  ATM_DELAY(8),
  ATM_NOTE_G4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 9" lead_bar8: lead bar 8 (E)
  ATM_NOTE_G4_,
  ATM_DELAY(4),
  ATM_NOTE_B4,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5_,
  ATM_DELAY(16),
  ATM_NOTE_D5,
  ATM_DELAY(8),
  ATM_NOTE_B4,
  ATM_DELAY(8),
  ATM_NOTE_G4_,
  ATM_DELAY(8),
  ATM_NOTE_E4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 10" chords: CH1 SQUARE: kick on every beat + offbeat arpeggiated chord stabs
  ATM_REPEAT(3, 12),   // 4x beat_Am
  ATM_REPEAT(3, 13),   // 4x beat_F
  ATM_REPEAT(3, 14),   // 4x beat_C
  ATM_REPEAT(3, 15),   // 4x beat_G
  ATM_REPEAT(3, 12),   // 4x beat_Am
  ATM_REPEAT(3, 13),   // 4x beat_F
  ATM_REPEAT(3, 15),   // 4x beat_G
  ATM_REPEAT(2, 16),   // 3x beat_E
  ATM_GOTO(16),   // -> beat_E
  ATM_GOTO(10),   // loop: restart this track (no STOP, no gap)

  //"Track 11" kick: kick: square wave with a fast downward pitch slide (4 ticks), then silence for 4 ticks
  ATM_ARP_OFF,
  ATM_SL_FRQ((uint8_t)-127),
  ATM_VOL(30),
  ATM_NOTE_D3_,
  ATM_DELAY(4),
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 12" beat_Am: 1 beat: kick + Am chord stab (8 ticks)
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 13" beat_F: 1 beat: kick + F chord stab (8 ticks)
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 14" beat_C: 1 beat: kick + C chord stab (8 ticks)
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x54, 0x20),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 15" beat_G: 1 beat: kick + G chord stab (8 ticks)
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_B3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 16" beat_E: 1 beat: kick + E chord stab (8 ticks)
  ATM_GOTO(11),   // -> kick
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_G3_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 17" bass: CH2 SAW bass: root changes via SET_TRANSPOSE (A2 is the base root)
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(20),   // -> bass_bar
  ATM_SET_TRA(3),
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

  //"Track 18" bass_beat: beat 1-3: duck on the kick, then root - octave - root
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(63),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 19" bass_beat4: beat 4 variation: root - root - octave
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(63),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 20" bass_bar: one bar of bass
  ATM_REPEAT(2, 18),   // 3x bass_beat
  ATM_GOTO(19),   // -> bass_beat4
  ATM_RETURN,

  //"Track 21" drums: CH3 NOISE: tempo + 8 bars of drums
  ATM_SET_TEMPO(35),
  ATM_REPEAT(2, 25),   // 3x drum_bar
  ATM_GOTO(27),   // -> drum_bar_fill
  ATM_REPEAT(2, 25),   // 3x drum_bar
  ATM_GOTO(27),   // -> drum_bar_fill
  ATM_GOTO(21),   // loop: restart this track (no STOP, no gap)

  //"Track 22" drum_A: beat 1/3: kick click, closed hat, open hat
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

  //"Track 23" drum_B: beat 2/4: clap, open hat
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(8),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 24" drum_pair: beats 1+2
  ATM_GOTO(22),   // -> drum_A
  ATM_GOTO(23),   // -> drum_B
  ATM_RETURN,

  //"Track 25" drum_bar: one bar of drums
  ATM_REPEAT(1, 24),   // 2x drum_pair
  ATM_RETURN,

  //"Track 26" drum_fill: snare roll (beat 4 of bar 4)
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

  //"Track 27" drum_bar_fill: bar 4: 3 beats + roll
  ATM_GOTO(24),   // -> drum_pair
  ATM_GOTO(22),   // -> drum_A
  ATM_GOTO(26),   // -> drum_fill
  ATM_RETURN,

};

#endif