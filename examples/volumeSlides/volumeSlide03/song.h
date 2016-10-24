#ifndef SONG_H
#define SONG_H

#define Song const uint8_t PROGMEM

Song music[] = {                // total song in bytes = 36
  //                            // setup bytes 11
  0x03,                         // Number of tracks
  0x00, 0x00,                   // Address of track 0
  0x03, 0x00,                   // Address of track 1
  0x0C, 0x00,                   // Address of track 2
  0x01,                         // Channel 0 entry track (PULSE)
  0x00,                         // Channel 1 entry track (SQUARE)
  0x00,                         // Channel 2 entry track (TRIANGLE)
  0x00,                         // Channel 3 entry track (NOISE)

  //"Track 0"                   // ticks = 0, bytes = 3
  0x40, 0,                      // FX: SET VOLUME: volume = 0
  0xFE,                         // RETURN (empty track used for silent channels)

  //"Track 1"                   // ticks = 768, bytes = 9
  0x40, 0,                      // FX: SET VOLUME: volume = 0
  0x42, 2, 24,                  // FX: VOLUME SLIDE ON: steps = 2 / every ticks = 24
  0xFD, 31, 2,                  // REPEAT: count = 32 - track = 2 (32 * 24 ticks)
  0xFE,                         // RETURN

  //"Track 2"                   // ticks = 24, bytes = 13
  0x00 +  37,                   // NOTE ON: note = 37
  0x9F +  4,                    // DELAY: ticks = 4
  0x00,                         // NOTE OFF
  0x9F +  4,                    // DELAY: ticks = 4
  0x00 +  41,                   // NOTE ON: note = 41
  0x9F +  4,                    // DELAY: ticks = 4
  0x00,                         // NOTE OFF
  0x9F +  4,                    // DELAY: ticks = 4
  0x00 +  44,                   // NOTE ON: note = 44
  0x9F +  4,                    // DELAY: ticks = 4
  0x00,                         // NOTE OFF
  0x9F +  4,                    // DELAY: ticks = 4
  0xFE,                         // RETURN
};

#endif
