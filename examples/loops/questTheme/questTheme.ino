#include <Arduino.h>
#include <Arduboy2.h>
#include "ATMconfig.h"
#include <ATMlib.h>
#include "bitmaps.h"
#include "song.h"


Arduboy2Base arduboy;
Sprites sprites;
ATMsynth ATM;

#include "arduboy_intro.h"          // the ARDUBOY letters, a note for every letter

byte introFrame = 0;

void setup() {
  arduboy.boot();
  // set the framerate of the game at 60 fps
  arduboy.setFrameRate(60);
  // let's make sure the sound was not muted in a previous sketch
  arduboy.audio.on();
}

void loop() {

  if (!(arduboy.nextFrame())) return;
  arduboy.pollButtons();
  arduboy.clear();
  if (introFrame < INTRO_FRAMES) arduboyIntro(introFrame++);        // ARDUBOY ...
  else
  {
    // ... then the T-arg logo and the song
    if (introFrame == INTRO_FRAMES)
    {
      ATM.play(questTheme);
      introFrame++;
    }
    sprites.drawSelfMasked(49, 20, T_arg, 0);
    if (arduboy.justPressed(A_BUTTON | B_BUTTON)) ATM.playPause();
  }
  arduboy.display();
}
