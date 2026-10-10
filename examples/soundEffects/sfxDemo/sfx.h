// ---------------------------------------------------------------------------
// "Poke" sound effects: write osc[ch].freq / osc[ch].vol directly, once per
// frame, from sfxUpdate(). No ATMlib track is involved, so the cost is only a
// few bytes per effect. Needs ATM_FUNC_MUTE 1 (see ATMconfig.h).
//
// Effect format (PROGMEM): 4-byte segments, then SFX_END.
//   SFX_SEG(m, f, sl, len)
//     m   volume shift 0..3: volume = min(63, framesLeft << m)  (fade-out)
//     f   start frequency, use SFX_HZ(hz)
//     sl  slide per frame, use SFX_SL(hz)  (+ up, - down, max about +-242 Hz)
//     len length in frames (60 fps)
// ---------------------------------------------------------------------------
#ifndef SFX_H
#define SFX_H
#include <ATMlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

extern ATMsynth ATM;

#define SFX_HZ(h)  ((uint16_t)((h) * 2097L / 1000))   // 31250 / 65536 = 0.4768 Hz per unit
#define SFX_SL(h)  ((int8_t)((h) * 524L / 1000))      // the slide is applied x4 per frame
#define SFX_SEG(m, f, sl, len) \
  (uint8_t)(len), (uint8_t)(sl), (uint8_t)((f) & 0xFF), (uint8_t)(((m) << 6) | ((f) >> 8))
#define SFX_END 0

static const uint8_t *sfxP;   // current segment, 0 = idle
static uint16_t sfxF;         // current frequency
static uint8_t  sfxT;         // frames left in the segment
static uint8_t  sfxC;         // channel in use

// The playroutine re-enables muted channels whenever the song loops, so the
// mute is re-asserted every frame. Changing it must not be interrupted.
static void sfxMute(uint8_t ch, bool on) {
  uint8_t sr = SREG; cli();
  if (on) ATM.muteChannel(ch); else ATM.unMuteChannel(ch);
  SREG = sr;
}

static void sfxLoad() {
  sfxT = pgm_read_byte(sfxP);
  if (!sfxT) {                       // end marker
    osc[sfxC].vol = 0;
    sfxMute(sfxC, false);
    sfxP = 0;
    return;
  }
  sfxF = pgm_read_word(sfxP + 2) & 0x3FFF;
}

bool sfxBusy() { return sfxP != 0; }

// Call once per frame (60 fps).
void sfxUpdate() {
  if (!sfxP) return;
  if (!sfxT) { sfxP += 4; sfxLoad(); if (!sfxP) return; }
  sfxMute(sfxC, true);
  uint16_t v = (uint16_t)sfxT << (pgm_read_byte(sfxP + 3) >> 6);
  if (v > 63) v = 63;
  if (sfxC == 3) v >>= 1;            // noise: osc[3].freq is the LFSR, never write it
  else osc[sfxC].freq = sfxF;
  osc[sfxC].vol = v;
  int16_t f = (int16_t)sfxF + (int8_t)pgm_read_byte(sfxP + 1) * 4;
  sfxF = f < 1 ? 1 : (f > 16383 ? 16383 : f);
  sfxT--;
}

void sfxStop() {
  if (!sfxP) return;
  sfxP = 0;
  osc[sfxC].vol = 0;
  sfxMute(sfxC, false);
}

void sfxPlay(const uint8_t *s, uint8_t ch) {
  if (sfxP && sfxC != ch) { osc[sfxC].vol = 0; sfxMute(sfxC, false); }
  sfxC = ch; sfxP = s; sfxLoad(); sfxUpdate();
}
#endif
