#include <Arduino.h>
#include <Arduboy2.h>
#include "ATMconfig.h"
#include <ATMlib.h>
#include "song.h"        // 30 small in-game loops

Arduboy2 arduboy;
ATMsynth ATM;

// Song names (in flash)
const char name0[] PROGMEM = "Meadow Waltz";
const char name1[] PROGMEM = "Lantern Lake";
const char name2[] PROGMEM = "Sunday Market";
const char name3[] PROGMEM = "Pixel Breeze";
const char name4[] PROGMEM = "Music Box Tide";
const char name5[] PROGMEM = "Coin Rush";
const char name6[] PROGMEM = "Bit Bounce";
const char name7[] PROGMEM = "Starlit Run";
const char name8[] PROGMEM = "Cave Crawl";
const char name9[] PROGMEM = "Victory Lap";
const char name10[] PROGMEM = "Turbo Street";
const char name11[] PROGMEM = "Sunset Strip";
const char name12[] PROGMEM = "Chrome Rider";
const char name13[] PROGMEM = "Marble Ruins";
const char name14[] PROGMEM = "Velvet Lounge";
const char name15[] PROGMEM = "Castle Gate";
const char name16[] PROGMEM = "Dungeon Dash";
const char name17[] PROGMEM = "Overworld Morning";
const char name18[] PROGMEM = "Fortress Run";
const char name19[] PROGMEM = "Pixel Parade";
const char name20[] PROGMEM = "Breadbin Beat";
const char name21[] PROGMEM = "SID Sunrise";
const char name22[] PROGMEM = "Loader Lullaby";
const char name23[] PROGMEM = "Raster Rain";
const char name24[] PROGMEM = "Joystick Jam";
const char name25[] PROGMEM = "Pocket Quest";
const char name26[] PROGMEM = "Handheld Hero";
const char name27[] PROGMEM = "Link Cable";
const char name28[] PROGMEM = "Brick Logic";
const char name29[] PROGMEM = "DMG Dawn";
const char* const songNames[] PROGMEM = {
  name0, name1, name2, name3, name4, name5, name6, name7, name8, name9, name10, name11, name12, name13, name14, name15, name16, name17, name18, name19, name20, name21, name22, name23, name24, name25, name26, name27, name28, name29
};

const char grp0[] PROGMEM = "Mixed";
const char grp1[] PROGMEM = "Chiptune";
const char grp2[] PROGMEM = "Sega style";
const char grp3[] PROGMEM = "NES style";
const char grp4[] PROGMEM = "C64 style";
const char grp5[] PROGMEM = "Game Boy";
const char* const groupNames[] PROGMEM = { grp0, grp1, grp2, grp3, grp4, grp5 };
// five songs per group, in the order above

// Song data (in flash)
const uint8_t* const songs[] PROGMEM = {
  meadowWaltz, lanternLake, sundayMarket, pixelBreeze, musicBoxTide,
  coinRush, bitBounce, starlitRun, caveCrawl, victoryLap,
  turboStreet, sunsetStrip, chromeRider, marbleRuins, velvetLounge,
  castleGate, dungeonDash, overworldMorning, fortressRun, pixelParade,
  breadbinBeat, sidSunrise, loaderLullaby, rasterRain, joystickJam,
  pocketQuest, handheldHero, linkCable, brickLogic, dmgDawn,
};

// Size of each song in bytes (computed by the compiler)
const uint16_t songBytes[] PROGMEM = {
  sizeof(meadowWaltz), sizeof(lanternLake), sizeof(sundayMarket), sizeof(pixelBreeze), sizeof(musicBoxTide),
  sizeof(coinRush), sizeof(bitBounce), sizeof(starlitRun), sizeof(caveCrawl), sizeof(victoryLap),
  sizeof(turboStreet), sizeof(sunsetStrip), sizeof(chromeRider), sizeof(marbleRuins), sizeof(velvetLounge),
  sizeof(castleGate), sizeof(dungeonDash), sizeof(overworldMorning), sizeof(fortressRun), sizeof(pixelParade),
  sizeof(breadbinBeat), sizeof(sidSunrise), sizeof(loaderLullaby), sizeof(rasterRain), sizeof(joystickJam),
  sizeof(pocketQuest), sizeof(handheldHero), sizeof(linkCable), sizeof(brickLogic), sizeof(dmgDawn),
};

const uint8_t NUM_SONGS = sizeof(songs) / sizeof(songs[0]);

uint8_t current = 0;
bool paused = false;

void playCurrent() {
  ATM.play((const uint8_t*)pgm_read_ptr(&songs[current]));
  paused = false;
}

void draw() {
  arduboy.clear();
  arduboy.setCursor(4, 4);
  arduboy.print(current + 1);
  arduboy.print(F("/"));
  arduboy.print(NUM_SONGS);
  arduboy.print(F("  "));
  arduboy.print((const __FlashStringHelper*)pgm_read_ptr(&groupNames[current / 5]));
  arduboy.setCursor(4, 24);
  arduboy.print((const __FlashStringHelper*)pgm_read_ptr(&songNames[current]));
  arduboy.setCursor(4, 34);
  arduboy.print(pgm_read_word(&songBytes[current]));
  arduboy.print(F(" bytes"));
  arduboy.setCursor(4, 52);
  arduboy.print(F("A next  B "));
  arduboy.print(paused ? F("play") : F("pause"));
  arduboy.display();
}

void setup() {
  arduboy.begin();
  arduboy.setFrameRate(30);
  arduboy.audio.on();
  playCurrent();
  draw();
}

void loop() {
  if (!arduboy.nextFrame()) return;
  arduboy.pollButtons();

  if (arduboy.justPressed(A_BUTTON)) {      // next song, wraps around
    current = (current + 1) % NUM_SONGS;
    playCurrent();
    draw();
  }
  if (arduboy.justPressed(B_BUTTON)) {      // pause / resume
    ATM.playPause();
    paused = !paused;
    draw();
  }
}
