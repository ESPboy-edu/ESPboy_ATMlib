// The 4 extra songs of the sound-effect demo (the 5th, Starlit Run, is in song.h).
#ifndef SONGS_H
#define SONGS_H

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

// "Pipe Dream" - NES style, C major (I vi IV V)
// pulse arpeggio hops over an octave-bouncing bass
// 104 bytes | tempo 42 (ticks per second) | loop 256 ticks
Song pipeDream[] = {          // total song in bytes = 104
  0x07,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x29, 0x00,                   // Address of track 2   //41
  0x41, 0x00,                   // Address of track 3   //65
  0x46, 0x00,                   // Address of track 4   //70
  0x4D, 0x00,                   // Address of track 5   //77
  0x54, 0x00,                   // Address of track 6   //84

  0x00,                         // Channel 0 entry track
  0x06,                         // Channel 1 entry track
  0x02,                         // Channel 2 entry track
  0x04,                         // Channel 3 entry track

  //"Track 0" lead  [24 bytes]
  ATM_SET_TEMPO(42),
  ATM_VOL(MV(34)),
  ATM_SL_VOL(MS(2)),
  ATM_SET_TRA(0),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-3),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-4),
  ATM_GOTO(1),
  ATM_ADD_TRA(2),
  ATM_GOTO(1),
  ATM_GOTO(0),

  //"Track 1" hop motif  [17 bytes]
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" bass  [24 bytes]
  ATM_VOL(MV(50)),
  ATM_SET_TRA(0),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-3),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA(2),
  ATM_REPEAT(3, 3),
  ATM_GOTO(2),

  //"Track 3" bass bounce  [5 bytes]
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL(MS(6)),
  ATM_REPEAT(15, 5),
  ATM_GOTO(4),

  //"Track 5" beat  [7 bytes]
  ATM_VOL(MV(50)),
  ATM_DELAY(8),
  ATM_VOL(MV(30)),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 6" silent  [1 bytes]
  ATM_STOP_CHAN,

};

// "Canyon Raid" - Atari 2600 style, A minor (Am G F E)
// wavering low square lead, saw pump bass, one noise thump per beat
// 102 bytes | tempo 36 (ticks per second) | loop 256 ticks
Song canyonRaid[] = {          // total song in bytes = 102
  0x07,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x19, 0x00,                   // Address of track 1   //25
  0x2C, 0x00,                   // Address of track 2   //44
  0x44, 0x00,                   // Address of track 3   //68
  0x47, 0x00,                   // Address of track 4   //71
  0x4E, 0x00,                   // Address of track 5   //78
  0x52, 0x00,                   // Address of track 6   //82

  0x06,                         // Channel 0 entry track
  0x00,                         // Channel 1 entry track
  0x02,                         // Channel 2 entry track
  0x04,                         // Channel 3 entry track

  //"Track 0" lead  [25 bytes]
  ATM_SET_TEMPO(36),
  ATM_VOL(MV(30)),
  ATM_VIB(3, 1),
  ATM_SET_TRA(0),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-1),
  ATM_GOTO(1),
  ATM_GOTO(0),

  //"Track 1" lead motif  [19 bytes]
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_NOTE_A3,
  ATM_DELAY(4),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_A3,
  ATM_DELAY(8),
  ATM_NOTE_E4,
  ATM_DELAY(8),
  ATM_NOTE_C4,
  ATM_DELAY(8),
  ATM_NOTE_D4,
  ATM_DELAY(8),
  ATM_NOTE_B3,
  ATM_DELAY(8),
  ATM_NOTE_E4,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" bass  [24 bytes]
  ATM_VOL(MV(52)),
  ATM_SET_TRA(0),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-1),
  ATM_REPEAT(3, 3),
  ATM_GOTO(2),

  //"Track 3" bass pump  [3 bytes]
  ATM_NOTE_A2,
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL(MS(9)),
  ATM_REPEAT(15, 5),
  ATM_GOTO(4),

  //"Track 5" beat  [4 bytes]
  ATM_VOL(MV(56)),
  ATM_DELAY(16),
  ATM_RETURN,

  //"Track 6" silent  [1 bytes]
  ATM_STOP_CHAN,

};

// "Green Zone" - Sega style, C major (C Bb F G)
// bright pulse lead over a syncopated bass and a hat-heavy beat
// 106 bytes | tempo 44 (ticks per second) | loop 256 ticks
Song greenZone[] = {          // total song in bytes = 106
  0x07,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x18, 0x00,                   // Address of track 1   //24
  0x2B, 0x00,                   // Address of track 2   //43
  0x43, 0x00,                   // Address of track 3   //67
  0x48, 0x00,                   // Address of track 4   //72
  0x4F, 0x00,                   // Address of track 5   //79
  0x56, 0x00,                   // Address of track 6   //86

  0x00,                         // Channel 0 entry track
  0x06,                         // Channel 1 entry track
  0x02,                         // Channel 2 entry track
  0x04,                         // Channel 3 entry track

  //"Track 0" lead  [24 bytes]
  ATM_SET_TEMPO(44),
  ATM_VOL(MV(36)),
  ATM_SL_VOL(MS(1)),
  ATM_SET_TRA(0),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_GOTO(1),
  ATM_ADD_TRA((uint8_t)-5),
  ATM_GOTO(1),
  ATM_ADD_TRA(2),
  ATM_GOTO(1),
  ATM_GOTO(0),

  //"Track 1" lead motif  [19 bytes]
  ATM_NOTE_G5,
  ATM_DELAY(4),
  ATM_NOTE_E5,
  ATM_DELAY(4),
  ATM_NOTE_C5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_C6,
  ATM_DELAY(8),
  ATM_NOTE_B5,
  ATM_DELAY(8),
  ATM_NOTE_G5,
  ATM_DELAY(8),
  ATM_NOTE_E5,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 2" bass  [24 bytes]
  ATM_VOL(MV(50)),
  ATM_SET_TRA(0),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-2),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA((uint8_t)-5),
  ATM_REPEAT(3, 3),
  ATM_ADD_TRA(2),
  ATM_REPEAT(3, 3),
  ATM_GOTO(2),

  //"Track 3" bass groove  [5 bytes]
  ATM_NOTE_C3,
  ATM_DELAY(8),
  ATM_NOTE_G3,
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL(MS(5)),
  ATM_REPEAT(15, 5),
  ATM_GOTO(4),

  //"Track 5" beat  [7 bytes]
  ATM_VOL(MV(52)),
  ATM_DELAY(8),
  ATM_VOL(MV(26)),
  ATM_DELAY(8),
  ATM_RETURN,

  //"Track 6" silent  [1 bytes]
  ATM_STOP_CHAN,

};

// "Neon Highway" - Sega style, D minor (Dm Bb F C)
// arpeggio lead, answering melody, driving 16th bass, four-on-the-floor
// 108 bytes | tempo 44 (ticks per second) | loop 256 ticks
Song neonHighway[] = {          // total song in bytes = 108
  0x06,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0   //0
  0x16, 0x00,                   // Address of track 1   //22
  0x2C, 0x00,                   // Address of track 2   //44
  0x44, 0x00,                   // Address of track 3   //68
  0x47, 0x00,                   // Address of track 4   //71
  0x4E, 0x00,                   // Address of track 5   //78

  0x00,                         // Channel 0 entry track
  0x01,                         // Channel 1 entry track
  0x02,                         // Channel 2 entry track
  0x04,                         // Channel 3 entry track

  //"Track 0" arp lead  [22 bytes]
  ATM_SET_TEMPO(44),
  ATM_VOL(MV(26)),
  ATM_SL_VOL(MS(1)),
  ATM_ARP(0x34, 0x20),
  ATM_NOTE_D5,
  ATM_DELAY(64),
  ATM_ARP(0x43, 0x20),
  ATM_NOTE_A4_,
  ATM_DELAY(64),
  ATM_NOTE_F5,
  ATM_DELAY(64),
  ATM_NOTE_C5,
  ATM_DELAY(64),
  ATM_GOTO(0),

  //"Track 1" melody  [22 bytes]
  ATM_VOL(MV(34)),
  ATM_SL_VOL(MS(1)),
  ATM_NOTE_D5,
  ATM_DELAY(32),
  ATM_NOTE_A5,
  ATM_DELAY(32),
  ATM_NOTE_D5,
  ATM_DELAY(32),
  ATM_NOTE_F5,
  ATM_DELAY(32),
  ATM_NOTE_C5,
  ATM_DELAY(32),
  ATM_NOTE_A5,
  ATM_DELAY(32),
  ATM_NOTE_C5,
  ATM_DELAY(32),
  ATM_NOTE_G5,
  ATM_DELAY(32),
  ATM_GOTO(1),

  //"Track 2" bass  [24 bytes]
  ATM_VOL(MV(50)),
  ATM_SET_TRA(0),
  ATM_REPEAT(15, 3),
  ATM_ADD_TRA((uint8_t)-4),
  ATM_REPEAT(15, 3),
  ATM_ADD_TRA(7),
  ATM_REPEAT(15, 3),
  ATM_ADD_TRA((uint8_t)-5),
  ATM_REPEAT(15, 3),
  ATM_GOTO(2),

  //"Track 3" bass 16th  [3 bytes]
  ATM_NOTE_D3,
  ATM_DELAY(4),
  ATM_RETURN,

  //"Track 4" drums  [7 bytes]
  ATM_SL_VOL(MS(5)),
  ATM_REPEAT(15, 5),
  ATM_GOTO(4),

  //"Track 5" beat  [13 bytes]
  ATM_VOL(MV(55)),
  ATM_DELAY(4),
  ATM_VOL(MV(20)),
  ATM_DELAY(4),
  ATM_VOL(MV(35)),
  ATM_DELAY(4),
  ATM_VOL(MV(20)),
  ATM_DELAY(4),
  ATM_RETURN,

};

#endif
