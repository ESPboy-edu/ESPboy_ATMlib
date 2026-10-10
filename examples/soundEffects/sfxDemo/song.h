#ifndef SONG_H
#define SONG_H

// Music volume for the demo, in percent, so the sound effects stand out.
// ATM_VOL(MV(n)) = n scaled by MUSIC_VOL_PCT (compile time, no extra bytes). 100 = original.
#ifndef MUSIC_VOL_PCT
#define MUSIC_VOL_PCT 60
#endif
#ifndef MV
#define MV(v) ((v) * MUSIC_VOL_PCT / 100)
#endif
// ATM_SL_VOL(MS(n)): the decay slide scaled the same way (n = size of the fade-out step per
// tick, at least 1), so notes keep their shape instead of dying sooner.
#ifndef MS
#define MS(n) ((uint8_t)-((((n) * MUSIC_VOL_PCT + 50) / 100) < 1 ? 1 : (((n) * MUSIC_VOL_PCT + 50) / 100)))
#endif

// Background loop for the sound-effect demo: "Starlit Run", 91 bytes, A minor, tempo 38.
// Channel 1 is silent, so the effects there never fight the music.
//  91 bytes | tempo 38 (ticks per second) | loop 256 ticks = 6.7 s
//  Every channel is one self-looping track (ATM_GOTO to itself); unused channels point at a 1-byte STOP track.
//  Remove ATM_SET_TEMPO (2 bytes) if your sketch sets the tempo itself.
// ===========================================================================
#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif


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
  ATM_VOL(MV(30)),
  ATM_SL_VOL(MS(1)),
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
  ATM_VOL(MV(56)),
  ATM_SL_VOL(MS(1)),
  ATM_NOTE_A2, ATM_DELAY(64),
  ATM_NOTE_F2, ATM_DELAY(64),
  ATM_NOTE_C3, ATM_DELAY(64),
  ATM_NOTE_G2, ATM_DELAY(64),
  ATM_GOTO(2),            // -> bass

  //"Track 3" beat  [7 bytes]
  ATM_VOL(MV(48)),
  ATM_DELAY(8),
  ATM_VOL(MV(42)),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL(MS(6)),
  ATM_REPEAT(15, 3),       // 16x beat
  ATM_GOTO(4),            // -> drums

  //"Track 5" silent  [1 bytes] - unused channel stops at once
  ATM_STOP_CHAN,

};

#endif