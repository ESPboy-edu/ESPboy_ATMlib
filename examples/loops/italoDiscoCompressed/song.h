#ifndef SONG_H
#define SONG_H

// ---------------------------------------------------------------------------
//  "Italo Night" - size-optimised rewrite (italoDiscoCompressed), hop-free self-loop
//  Same notes, volumes, slides and loop as Italo Night: only the track layout is smaller.
//  (Verified tick by tick against the original: all note events and the volume/frequency
//  timeline of the 4 channels are identical; the loop seam has no silent tick.)
//
//  Play with:   #include "song.h"      ATM.play(italoDiscoCompressed);
//
//  Timing : ATM_SET_TEMPO(35).  1 beat = 16 ticks, 1 bar = 64 ticks, 8 bars = 512 ticks.
//           Every channel sums to exactly 512 ticks; each entry track ends with ATM_GOTO(itself).
//  Chords : Am | F | C | G | Am | F | G | E
//  Compression (see README, "Making a song smaller"):
//   - one track per channel; every single-use bar track is inlined
//   - drums: REPEAT of a 2-beat pair, and the 4-bar half (with its roll) is played twice by one REPEAT
//   - the kick head is shared by all chord stabs; the 4th beat of the E bar is the same beat track
//   - bass: one bar track used 8 times with ATM_SET_TRA; beats share one head (silence + first root)
//  Note   : track 0 is only CH3's entry point and is never called by another track.
//  Pitch  : ATMlib's note names sound about an octave above their label.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif



Song italoDiscoCompressed[] = {     // total song bytes = 365
  0x10,                       // Number of tracks
  0x00, 0x00,                 // Address of track 0      0   drums
  0x07, 0x00,                 // Address of track 1      7   drum_half
  0x27, 0x00,                 // Address of track 2     39   drum_A
  0x37, 0x00,                 // Address of track 3     55   drum_pair
  0x44, 0x00,                 // Address of track 4     68   lead
  0xBA, 0x00,                 // Address of track 5    186   chords
  0xD4, 0x00,                 // Address of track 6    212   kick
  0xE3, 0x00,                 // Address of track 7    227   beat_Am
  0xEB, 0x00,                 // Address of track 8    235   beat_F
  0xF3, 0x00,                 // Address of track 9    243   beat_C
  0xFB, 0x00,                 // Address of track 10   251   beat_G
  0x03, 0x01,                 // Address of track 11   259   beat_E
  0x0B, 0x01,                 // Address of track 12   267   bass
  0x2F, 0x01,                 // Address of track 13   303   bass_head
  0x37, 0x01,                 // Address of track 14   311   bass_beat
  0x3E, 0x01,                 // Address of track 15   318   bass_bar

  0x04,                         // CH0 entry -> track 4 (lead)
  0x05,                         // CH1 entry -> track 5 (chords)
  0x0C,                         // CH2 entry -> track 12 (bass)
  0x00,                         // CH3 entry -> track 0 (drums)

  //"Track 0" drums  [7b]
  ATM_SET_TEMPO(35),
  ATM_REPEAT(1, 1),   // 2x drum_half
  ATM_GOTO(0),   // -> drums  (loop: this very track)

  //"Track 1" drum_half  [32b]
  ATM_REPEAT(6, 3),   // 7x drum_pair
  ATM_GOTO(2),   // -> drum_A
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

  //"Track 2" drum_A  [16b]
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

  //"Track 3" drum_pair  [13b]
  ATM_GOTO(2),   // -> drum_A
  ATM_VOL(40),
  ATM_SL_VOL((uint8_t)-5),
  ATM_DELAY(8),
  ATM_VOL(26),
  ATM_SL_VOL((uint8_t)-3),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" lead  [118b]
  ATM_VOL(36),
  ATM_SL_VOL((uint8_t)-2),
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
  ATM_GOTO(4),   // -> lead  (loop: this very track)

  //"Track 5" chords  [26b]
  ATM_REPEAT(3, 7),   // 4x beat_Am
  ATM_REPEAT(3, 8),   // 4x beat_F
  ATM_REPEAT(3, 9),   // 4x beat_C
  ATM_REPEAT(3, 10),   // 4x beat_G
  ATM_REPEAT(3, 7),   // 4x beat_Am
  ATM_REPEAT(3, 8),   // 4x beat_F
  ATM_REPEAT(3, 10),   // 4x beat_G
  ATM_REPEAT(3, 11),   // 4x beat_E
  ATM_GOTO(5),   // -> chords  (loop: this very track)

  //"Track 6" kick  [15b]
  ATM_ARP_OFF,
  ATM_SL_FRQ((uint8_t)-127),
  ATM_VOL(30),
  ATM_NOTE_D3_,
  ATM_DELAY(4),
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_SL_VOL((uint8_t)-3),
  ATM_VOL(24),
  ATM_RETURN,

  //"Track 7" beat_Am  [8b]
  ATM_GOTO(6),   // -> kick
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 8" beat_F  [8b]
  ATM_GOTO(6),   // -> kick
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 9" beat_C  [8b]
  ATM_GOTO(6),   // -> kick
  ATM_ARP(0x54, 0x20),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 10" beat_G  [8b]
  ATM_GOTO(6),   // -> kick
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_B3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 11" beat_E  [8b]
  ATM_GOTO(6),   // -> kick
  ATM_ARP(0x35, 0x20),
  ATM_NOTE_G3_,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 12" bass  [36b]
  ATM_SL_VOL((uint8_t)-5),
  ATM_SET_TRA(0),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA(3),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA(0),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-4),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-2),
  ATM_GOTO(15),   // -> bass_bar
  ATM_SET_TRA((uint8_t)-5),
  ATM_GOTO(15),   // -> bass_bar
  ATM_GOTO(12),   // -> bass  (loop: this very track)

  //"Track 13" bass_head  [8b]
  ATM_VOL(0),
  ATM_DELAY(4),
  ATM_VOL(63),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 14" bass_beat  [7b]
  ATM_GOTO(13),   // -> bass_head
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 15" bass_bar  [10b]
  ATM_REPEAT(2, 14),   // 3x bass_beat
  ATM_GOTO(13),   // -> bass_head
  ATM_NOTE_A2,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_RETURN,

};

#endif
