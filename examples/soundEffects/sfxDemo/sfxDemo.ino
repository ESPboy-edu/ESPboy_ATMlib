// ---------------------------------------------------------------------------
// ATMlib sound-effect demo: 45 "poke" effects and the same 45 as ATM.playSfx()
// effects, over 5 looping songs.
//   A      play the selected poke effect        LEFT   next poke effect
//   B      play the selected playSfx effect     RIGHT  next playSfx effect
//   UP     next song                            DOWN   pause / resume the song
// Both lists hold the same effects in the same order (Jump ... Magic, then 10 NES,
// 10 Atari and 5 Sega style), so you can A/B the two methods.
// The screen shows each effect's name and its size in bytes.
// ATMconfig.h must keep ATM_FUNC_MUTE and ATM_FUNC_UNMUTE (poke) and
// ATM_FUNC_SFX (playSfx) switched on.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <Arduboy2.h>
#include "ATMconfig.h"
#include <ATMlib.h>
#include "song.h"
#include "songs.h"
#include "pokeSfx.h"
#include "atmSfx.h"

Arduboy2 arduboy;
ATMsynth ATM;

struct SfxInfo {
  char name[10];
  const uint8_t *data;
  uint8_t size;     // bytes in flash, including the end marker
  uint8_t ch;       // channel the effect plays on
};

#define FX(n, d, c) { n, d, sizeof(d), c }

#define COUNT 45

static const SfxInfo pokeList[] PROGMEM = {
  FX("Jump", pJump, 1),
  FX("Coin", pCoin, 1),
  FX("Laser", pLaser, 0),
  FX("Hit", pHit, 3),
  FX("Explode", pExplode, 3),
  FX("PowerUp", pPowerUp, 1),
  FX("Select", pSelect, 0),
  FX("Fail", pFail, 1),
  FX("Blip", pBlip, 0),
  FX("Whoosh", pWhoosh, 0),
  FX("Siren", pSiren, 0),
  FX("Gem", pGem, 0),
  FX("Bounce", pBounce, 2),
  FX("Warp", pWarp, 1),
  FX("PowerDown", pPowerDn, 1),
  FX("Alarm", pAlarm, 0),
  FX("Click", pClick, 0),
  FX("Thud", pThud, 2),
  FX("Zip", pZip, 0),
  FX("Magic", pMagic, 0),
  FX("NES Pick", pNPick, 0),
  FX("NES Shot", pNShot, 0),
  FX("NES Stomp", pNStomp, 1),
  FX("NES 1UP", pN1up, 1),
  FX("NES Pipe", pNPipe, 2),
  FX("NES Fire", pNFire, 3),
  FX("NES Block", pNBlock, 0),
  FX("NES Dash", pNDash, 3),
  FX("NES Dead", pNDead, 1),
  FX("NES Beep", pNBeep, 1),
  FX("ATA Pew", pTPew, 0),
  FX("ATA Crash", pTCrash, 3),
  FX("ATA Buzz", pTBuzz, 2),
  FX("ATA Bomb", pTBomb, 2),
  FX("ATA Blip", pTBlip, 0),
  FX("ATA Zap", pTZap, 0),
  FX("ATA Lose", pTLose, 2),
  FX("ATA Rumb", pTRumb, 2),
  FX("ATA Tick", pTTick, 0),
  FX("ATA Win", pTWin, 0),
  FX("SEG Ring", pGRing, 0),
  FX("SEG Spin", pGSpin, 1),
  FX("SEG Bump", pGBump, 1),
  FX("SEG Boing", pGBoing, 0),
  FX("SEG Chime", pGChime, 0),
};

static const SfxInfo atmList[] PROGMEM = {
  FX("Jump", aJump, 1),
  FX("Coin", aCoin, 1),
  FX("Laser", aLaser, 0),
  FX("Hit", aHit, 3),
  FX("Explode", aExplode, 3),
  FX("PowerUp", aPowerUp, 1),
  FX("Select", aSelect, 0),
  FX("Fail", aFail, 1),
  FX("Blip", aBlip, 0),
  FX("Whoosh", aWhoosh, 0),
  FX("Siren", aSiren, 0),
  FX("Gem", aGem, 0),
  FX("Bounce", aBounce, 2),
  FX("Warp", aWarp, 1),
  FX("PowerDown", aPowerDn, 1),
  FX("Alarm", aAlarm, 0),
  FX("Click", aClick, 0),
  FX("Thud", aThud, 2),
  FX("Zip", aZip, 0),
  FX("Magic", aMagic, 0),
  FX("NES Pick", aNPick, 0),
  FX("NES Shot", aNShot, 0),
  FX("NES Stomp", aNStomp, 1),
  FX("NES 1UP", aN1up, 1),
  FX("NES Pipe", aNPipe, 2),
  FX("NES Fire", aNFire, 3),
  FX("NES Block", aNBlock, 0),
  FX("NES Dash", aNDash, 3),
  FX("NES Dead", aNDead, 1),
  FX("NES Beep", aNBeep, 1),
  FX("ATA Pew", aTPew, 0),
  FX("ATA Crash", aTCrash, 3),
  FX("ATA Buzz", aTBuzz, 2),
  FX("ATA Bomb", aTBomb, 2),
  FX("ATA Blip", aTBlip, 0),
  FX("ATA Zap", aTZap, 0),
  FX("ATA Lose", aTLose, 2),
  FX("ATA Rumb", aTRumb, 2),
  FX("ATA Tick", aTTick, 0),
  FX("ATA Win", aTWin, 0),
  FX("SEG Ring", aGRing, 0),
  FX("SEG Spin", aGSpin, 1),
  FX("SEG Bump", aGBump, 1),
  FX("SEG Boing", aGBoing, 0),
  FX("SEG Chime", aGChime, 0),
};

struct SongInfo {
  char name[18];
  const uint8_t *data;
  uint8_t size;
};

#define SG(n, d) { n, d, sizeof(d) }

static const SongInfo songList[] PROGMEM = {
  SG("Starlit Run", starlitRun),
  SG("NES Pipe Dream", pipeDream),
  SG("ATA Canyon Raid", canyonRaid),
  SG("SEG Green Zone", greenZone),
  SG("SEG Neon Highway", neonHighway),
};
#define SONGS (sizeof(songList) / sizeof(songList[0]))

static uint8_t pokeSel = 0, atmSel = 0, songSel = 0;

// While the song is paused the playroutine zeroes the volume of every channel that has no
// playSfx running, which would cut a poked effect into 38 Hz pieces. So a poke effect started
// during the pause first claims its channel with a silent 1 s playSfx (a few bytes); the
// playroutine then leaves that channel alone. Not needed while the song plays.
ATM_SFX_TRACK(pokeHold, ATM_VOL(0), ATM_DELAY(40));
static bool songPaused = false;

static void drawRow(uint8_t y, const char *title, const SfxInfo *list, uint8_t sel) {
  SfxInfo s;
  memcpy_P(&s, &list[sel], sizeof s);
  arduboy.setCursor(0, y);
  arduboy.print(title);
  arduboy.setCursor(104, y);
  if (sel + 1 < 10) arduboy.print('0');
  arduboy.print(sel + 1);
  arduboy.print('/');
  arduboy.print(COUNT);
  arduboy.setCursor(0, y + 9);
  arduboy.print(s.name);
  arduboy.setCursor(84, y + 9);
  if (s.size < 10) arduboy.print(' ');
  arduboy.print(s.size);
  arduboy.print(F(" bytes"));
}

static void playSong() {
  SongInfo g;
  memcpy_P(&g, &songList[songSel], sizeof g);
  sfxStop();                 // ATM.play() resets the channel mutes
  ATM.play(g.data);          // also clears a pause
  songPaused = false;
}

void setup() {
  arduboy.begin();
  arduboy.setFrameRate(60);
  arduboy.audio.on();
  playSong();
}

void loop() {
  if (!(arduboy.nextFrame())) return;
  sfxUpdate();                         // poke effects advance once per frame
  arduboy.pollButtons();

  if (arduboy.justPressed(LEFT_BUTTON))  pokeSel = (pokeSel + 1) % COUNT;
  if (arduboy.justPressed(RIGHT_BUTTON)) atmSel  = (atmSel + 1) % COUNT;
  if (arduboy.justPressed(UP_BUTTON)) { songSel = (songSel + 1) % SONGS; playSong(); }

  if (arduboy.justPressed(A_BUTTON)) {
    SfxInfo s;
    memcpy_P(&s, &pokeList[pokeSel], sizeof s);
    if (songPaused) ATM.playSfx(pokeHold, s.ch);   // claim the channel (see above)
    sfxPlay(s.data, s.ch);
  }
  if (arduboy.justPressed(B_BUTTON)) {
    SfxInfo s;
    memcpy_P(&s, &atmList[atmSel], sizeof s);
    sfxStop();                         // release the channel a poke effect had muted
    ATM.playSfx(s.data, s.ch);
  }
  if (arduboy.justPressed(DOWN_BUTTON)) { ATM.playPause(); songPaused = !songPaused; }

  arduboy.clear();
  drawRow(0,  "A poke (LEFT)", pokeList, pokeSel);
  arduboy.drawFastHLine(0, 20, 128);
  drawRow(23, "B playSfx (RIGHT)", atmList, atmSel);
  arduboy.drawFastHLine(0, 43, 128);

  SongInfo g;
  memcpy_P(&g, &songList[songSel], sizeof g);
  arduboy.setCursor(0, 46);
  arduboy.print(F("UP:"));
  arduboy.print(g.name);
  arduboy.setCursor(0, 56);
  arduboy.print(songPaused ? F("DOWN:resume") : F("DOWN:pause"));
  arduboy.setCursor(84, 56);
  if (g.size < 100) arduboy.print(' ');
  arduboy.print(g.size);
  arduboy.print(F(" bytes"));
  arduboy.display();
}
