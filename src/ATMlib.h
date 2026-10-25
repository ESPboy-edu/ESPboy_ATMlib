#ifndef _ATMLIB_H_
#define _ATMLIB_H_
#include <stddef.h>
#include <inttypes.h>
#include <Arduino.h>
#include <sigma_delta.h>

#include "ATMcmds.h"
#if __has_include("ATMconfig.h")
  #include "ATMconfig.h"
#endif


#ifndef ATM_FX_VOLUME
#define ATM_FX_VOLUME 1
#endif
#ifndef ATM_FX_VOL_SLIDE
#define ATM_FX_VOL_SLIDE 1
#endif
#ifndef ATM_FX_FREQ_SLIDE
#define ATM_FX_FREQ_SLIDE 1
#endif
#ifndef ATM_FX_ARP
#define ATM_FX_ARP 1
#endif
#ifndef ATM_FX_NOISE
#define ATM_FX_NOISE 1
#endif
#ifndef ATM_FX_TRANSPOSE
#define ATM_FX_TRANSPOSE 1
#endif
#ifndef ATM_FX_TREMOLO
#define ATM_FX_TREMOLO 1
#endif
#ifndef ATM_FX_VIBRATO
#define ATM_FX_VIBRATO 1
#endif
#ifndef ATM_FX_GLISSANDO
#define ATM_FX_GLISSANDO 1
#endif
#ifndef ATM_FX_NOTE_CUT
#define ATM_FX_NOTE_CUT 1
#endif
#ifndef ATM_FX_CUE
#define ATM_FX_CUE 1
#endif
#ifndef ATM_FX_TEMPO
#define ATM_FX_TEMPO 1
#endif
#ifndef ATM_FX_GOTO_ADV
#define ATM_FX_GOTO_ADV 1
#endif
#ifndef ATM_FX_STOP
#define ATM_FX_STOP 1
#endif

#ifndef ATM_FUNC_STOP
#define ATM_FUNC_STOP 1
#endif
#ifndef ATM_FUNC_PAUSE
#define ATM_FUNC_PAUSE 1
#endif
#ifndef ATM_FUNC_RESUME
#define ATM_FUNC_RESUME 1
#endif
#ifndef ATM_FUNC_PLAYPAUSE
#define ATM_FUNC_PLAYPAUSE 1
#endif
#ifndef ATM_FUNC_MUTE
#define ATM_FUNC_MUTE 1
#endif
#ifndef ATM_FUNC_UNMUTE
#define ATM_FUNC_UNMUTE 1
#endif
#ifndef ATM_FUNC_SFX
#define ATM_FUNC_SFX 1
#endif
#ifndef ATM_FUNC_TEMPO
#define ATM_FUNC_TEMPO 1
#endif
#ifndef ATM_FUNC_CUE
#define ATM_FUNC_CUE 1
#endif

#ifndef ATM_OCTAVE_2
#define ATM_OCTAVE_2 1
#endif
#ifndef ATM_OCTAVE_3
#define ATM_OCTAVE_3 1
#endif
#ifndef ATM_OCTAVE_4
#define ATM_OCTAVE_4 1
#endif
#ifndef ATM_OCTAVE_5
#define ATM_OCTAVE_5 1
#endif
#ifndef ATM_OCTAVE_6
#define ATM_OCTAVE_6 1
#endif
#ifndef ATM_OCTAVE_7
#define ATM_OCTAVE_7 1
#endif

// Глубина стека вызовов (настраиваемая)[cite: 21]
#ifndef ATM_STACK_DEPTH
#define ATM_STACK_DEPTH 7
#endif

#if ATM_FUNC_PLAYPAUSE && (!ATM_FUNC_PAUSE || !ATM_FUNC_RESUME)
#error ATM_FUNC_PLAYPAUSE needs ATM_FUNC_PAUSE and ATM_FUNC_RESUME
#endif

#define CH_ZERO             0
#define CH_ONE              1
#define CH_TWO              2
#define CH_THREE            3

extern byte trackCount;
extern const word *trackList;
extern const byte *trackBase;
extern uint8_t pcm;

class ATMsynth {
  public:
    ATMsynth() {};

    static void play(const byte *song);
    static bool isPlay();

#if ATM_FUNC_PLAYPAUSE
    static void playPause();
#endif
#if ATM_FUNC_PAUSE
    static void pause();
#endif
#if ATM_FUNC_RESUME
    static void resume();
#endif
#if ATM_FUNC_STOP
    static void stop();
#endif
#if ATM_FUNC_MUTE
    static void muteChannel(byte ch);
#endif
#if ATM_FUNC_UNMUTE
    static void unMuteChannel(byte ch);
#endif
#if ATM_FUNC_SFX
    static void playSfx(const byte *track, byte ch);
#endif
#if ATM_FUNC_TEMPO
    static void setTempo(byte t);
#endif
#if ATM_FUNC_CUE
    static uint8_t check();
    static uint8_t check(uint8_t id);
#endif
};

typedef struct {
  uint8_t  vol;
  uint16_t freq;
  uint16_t phase;
} osc_t;

typedef osc_t Oscillator;

extern osc_t osc[4];

uint16_t read_vle(const byte **pp);
static inline const byte *getTrackPointer(byte track);

extern void IRAM_ATTR ATM_playroutine();
#endif