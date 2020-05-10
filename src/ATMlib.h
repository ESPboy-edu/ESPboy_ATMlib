#ifndef _ATMLIB_H_
#define _ATMLIB_H_
#include <stddef.h>
#include <inttypes.h>
#include <Arduino.h>

#define CH_ZERO             0
#define CH_ONE              1
#define CH_TWO              2
#define CH_THREE            3

extern byte trackCount;
extern const word *trackList;
extern const byte *trackBase;
extern uint8_t pcm;

extern bool half;

class ATMsynth {

  public:
    ATMsynth() {};

    // Load and play specified song
    void play(const byte *song);

    // Play or Pause playback
    void playPause();

    // Stop playback (unloads song)
    void stop();

    void muteChannel(byte ch);

    void unMuteChannel(byte ch);
};


// oscillator structure
typedef struct {
  uint8_t  vol;
  uint16_t freq;
  uint16_t phase;
} osc_t;

typedef osc_t Oscillator;

extern osc_t osc[4];


uint16_t read_vle(const byte **pp);
static inline const byte *getTrackPointer(byte track);



extern void ATM_playroutine() asm("ATM_playroutine");

#ifndef AB_ALTERNATE_WIRING
#define ATMLIB_CONSTRUCT_ISR(TARGET_REGISTER) \
uint16_t __attribute__((used)) cia, __attribute__((used)) cia_count;    /* uint16_t cia, cia_count;                             */ \
ISR(TIMER4_OVF_vect, ISR_NAKED) {                                       /* ISR(TIMER4_OVF_vect) {                               */ \
  asm volatile( \
                "push r2                                            \n"                                                            \
                "in   r2,                    __SREG__               \n"                                                            \
                "push r18                                           \n"                                                            \
                "lds  r18,                   half                   \n" /* half = !half;                                        */ \
                "com  r18                                           \n"                                                            \
                "sts  half,                  r18                    \n"                                                            \
                "breq 1f                                            \n" /* if (half) return;                                    */ \
                "rjmp 4f                                            \n"                                                            \
                "1:                                                 \n"                                                            \
                "push r27                                           \n"                                                            \
                "push r26                                           \n"                                                            \
                "push r0                                            \n"                                                            \
                "push r1                                            \n"                                                            \
                \
                "lds  r18,                   osc+2*%[mul]+%[fre]    \n" /* osc[2].phase += osc[2].freq;// update triangle phase */ \
                "lds  r0,                    osc+2*%[mul]+%[pha]    \n"                                                            \
                "add  r0,                    r18                    \n"                                                            \
                "sts  osc+2*%[mul]+%[pha],   r0                     \n"                                                            \
                "lds  r18,                   osc+2*%[mul]+%[fre]+1  \n"                                                            \
                "lds  r1,                    osc+2*%[mul]+%[pha]+1  \n"                                                            \
                "adc  r1,                    r18                    \n"                                                            \
                "sts  osc+2*%[mul]+%[pha]+1, r1                     \n"                                                            \
                \
                "mov  r27,                   r1                     \n" /* int8_t phase2 = osc[2].phase >> 8;                   */ \
                "sbrc r27,                   7                      \n" /* if (phase2 < 0) phase2 = ~phase2;                    */ \
                "com  r27                                           \n"                                                            \
                "lsl  r27                                           \n" /* phase2 <<= 1;                                        */ \
                "lds  r26,                   osc+2*%[mul]+%[vol]    \n" /* int8_t vol2 = osc[2].vol;                            */ \
                "subi r27,                   128                    \n" /* phase2 -= 128;                                       */ \
                "muls r27,                   r26                    \n"                                                            \
                "lsl  r1                                            \n"                                                            \
                "mov  r26,                   r1                     \n" /* int8_t vol = ((phase2 * vol2) << 1) >> 8;            */ \
                \
                "lds  r18,                   osc+0*%[mul]+%[fre]    \n" /* osc[0].phase += osc[0].freq; // update pulse phase   */ \
                "lds  r0,                    osc+0*%[mul]+%[pha]    \n"                                                            \
                "add  r0,                    r18                    \n"                                                            \
                "sts  osc+0*%[mul]+%[pha],   r0                     \n"                                                            \
                "lds  r1,                    osc+0*%[mul]+%[fre]+1  \n"                                                            \
                "lds  r18,                   osc+0*%[mul]+%[pha]+1  \n"                                                            \
                "adc  r18,                   r1                     \n"                                                            \
                "sts  osc+0*%[mul]+%[pha]+1, r18                    \n"                                                            \
                \
                \
                "lds  r27,                   osc+0*%[mul]+%[vol]    \n" /* int8_t vol0 = osc[0].vol;                            */ \
                "cpi  r18,                   192                    \n" /* if (uint8_t(osc[0].phase >> 8) >= 192) vol0 = -vol0; */ \
                "brcs 2f                                            \n"                                                            \
                "neg  r27                                           \n"                                                            \
                "2:                                                 \n"                                                            \
                "add  r26,                   r27                    \n" /* vol += vol0;                                         */ \
                \
                "lds  r18,                   osc+1*%[mul]+%[fre]    \n" /* osc[1].phase += osc[1].freq; // update square phase  */ \
                "lds  r0,                    osc+1*%[mul]+%[pha]    \n"                                                            \
                "add  r0,                    r18                    \n"                                                            \
                "sts  osc+1*%[mul]+%[pha],   r0                     \n"                                                            \
                "lds  r18,                   osc+1*%[mul]+%[fre]+1  \n"                                                            \
                "lds  r1,                    osc+1*%[mul]+%[pha]+1  \n"                                                            \
                "adc  r1,                    r18                    \n"                                                            \
                "sts  osc+1*%[mul]+%[pha]+1, r1                     \n"                                                            \
                \
                "lds  r27,                   osc+1*%[mul]+%[vol]    \n" /* int8_t vol1 = osc[1].vol;                            */ \
                "sbrc r1,                    7                      \n" /* if (osc[1].phase & 0x8000) vol1 = -vol1;             */ \
                "neg  r27                                           \n"                                                            \
                "add  r26,                   r27                    \n" /* vol += vol1;                                         */ \
                \
                "ldi  r27,                   1                      \n"                                                            \
                "lds  r0,                    osc+3*%[mul]+%[fre]    \n" /* uint16_t freq = osc[3].freq; //noise frequency       */ \
                "lds  r1,                    osc+3*%[mul]+%[fre]+1  \n"                                                            \
                "add  r0,                    r0                     \n" /* freq <<= 1;                                          */ \
                "adc  r1,                    r1                     \n"                                                            \
                "sbrc r1,                    7                      \n" /* if (freq & 0x8000) freq ^= 1;                        */ \
                "eor  r0,                    r27                    \n"                                                            \
                "sbrc r1,                    6                      \n" /* if (freq & 0x4000) freq ^= 1;                        */ \
                "eor  r0,                    r27                    \n"                                                            \
                "sts  osc+3*%[mul]+%[fre],   r0                     \n" /* osc[3].freq = freq;                                  */ \
                "sts  osc+3*%[mul]+%[fre]+1, r1                     \n"                                                            \
                \
                "lds  r27,                   osc+3*%[mul]+%[vol]    \n" /* int8_t vol3 = osc[3].vol;                            */ \
                "sbrc r1,                    7                      \n" /* if (freq & 0xc8000) vol3 = -vol3;                    */ \
                "neg  r27                                           \n"                                                            \
                "add  r26,                   r27                    \n" /* vol += vol3;                                         */ \
                \
                "lds  r27,                   pcm                    \n" /* reg = vol + pcm;                                     */ \
                "add  r26,                   r27                    \n"                                                            \
                "sts  %[reg],                r26                    \n"                                                            \
                \
                "lds  r27,                   cia_count+1            \n" /* if (--cia_count) return;                             */ \
                "lds  r26,                   cia_count              \n"                                                            \
                "sbiw r26,                   1                      \n"                                                            \
                "breq call_playroutine                              \n"                                                            \
                "sts  cia_count+1,           r27                    \n"                                                            \
                "sts  cia_count,             r26                    \n"                                                            \
                "rjmp 3f                                            \n"                                                            \
                \
                "call_playroutine:                                  \n"                                                            \
                \
                "lds  r27, cia+1                                    \n" /* cia_count = cia;                                     */ \
                "lds  r26, cia                                      \n"                                                            \
                "sts  cia_count+1,           r27                    \n"                                                            \
                "sts  cia_count,             r26                    \n"                                                            \
                \
                "sei                                                \n" /* sei();                                               */ \
                "push r19                                           \n"                                                            \
                "push r20                                           \n"                                                            \
                "push r21                                           \n"                                                            \
                "push r22                                           \n"                                                            \
                "push r23                                           \n"                                                            \
                "push r24                                           \n"                                                            \
                "push r25                                           \n"                                                            \
                "push r30                                           \n"                                                            \
                "push r31                                           \n"                                                            \
                \
                "clr  r1                                            \n"                                                            \
                "call ATM_playroutine                               \n" /* ATM_playroutine();                                   */ \
                \
                "pop  r31                                           \n" /* }                                                    */ \
                "pop  r30                                           \n"                                                            \
                "pop  r25                                           \n"                                                            \
                "pop  r24                                           \n"                                                            \
                "pop  r23                                           \n"                                                            \
                "pop  r22                                           \n"                                                            \
                "pop  r21                                           \n"                                                            \
                "pop  r20                                           \n"                                                            \
                "pop  r19                                           \n"                                                            \
                "3:                                                 \n"                                                            \
                "pop  r1                                            \n"                                                            \
                "pop  r0                                            \n"                                                            \
                "pop  r26                                           \n"                                                            \
                "pop  r27                                           \n"                                                            \
                "4:                                                 \n"                                                            \
                "pop  r18                                           \n"                                                            \
                "out  __SREG__,              r2                     \n"                                                            \
                "pop  r2                                            \n"                                                            \
                "reti                                               \n"                                                            \
                : \
                : [reg]  "M" _SFR_MEM_ADDR(TARGET_REGISTER), \
                  [mul]  "M" (sizeof(Oscillator)), \
                  [pha]  "M" (offsetof(Oscillator, phase)), \
                  [fre]  "M" (offsetof(Oscillator, freq)), \
                  [vol]  "M" (offsetof(Oscillator, vol)) \
              ); \
}
#else
#define ATMLIB_CONSTRUCT_ISR(TARGET_REGISTER,TARGET_REGISTER2) \
uint16_t __attribute__((used)) cia, __attribute__((used)) cia_count; \
ISR(TIMER4_OVF_vect, ISR_NAKED) { \
  asm volatile( \
                "push r2                                            \n" \
                "in   r2,                    __SREG__               \n" \
                "push r18                                           \n" \
                "lds  r18,                   half                   \n" \
                "com  r18                                           \n" \
                "sts  half,                  r18                    \n" \
                "breq 1f                                            \n" \
                "rjmp 3f                                            \n" \
                "1:                                                 \n" \
                "push r27                                           \n" \
                "push r26                                           \n" \
                "push r0                                            \n" \
                "push r1                                            \n" \
                \
                "lds  r18,                   osc+2*%[mul]+%[fre]    \n" \
                "lds  r0,                    osc+2*%[mul]+%[pha]    \n" \
                "add  r0,                    r18                    \n" \
                "sts  osc+2*%[mul]+%[pha],   r0                     \n" \
                "lds  r18,                   osc+2*%[mul]+%[fre]+1  \n" \
                "lds  r1,                    osc+2*%[mul]+%[pha]+1  \n" \
                "adc  r1,                    r18                    \n" \
                "sts  osc+2*%[mul]+%[pha]+1, r1                     \n" \
                \
                "mov  r27,                   r1                     \n" \
                "sbrc r27,                   7                      \n" \
                "com  r27                                           \n" \
                "lsl  r27                                           \n" \
                "lds  r26,                   osc+2*%[mul]+%[vol]    \n" \
                "subi r27,                   128                    \n" \
                "muls r27,                   r26                    \n" \
                "lsl  r1                                            \n" \
                "mov  r26,                   r1                     \n" \
                \
                "lds  r18,                   osc+0*%[mul]+%[fre]    \n" \
                "lds  r0,                    osc+0*%[mul]+%[pha]    \n" \
                "add  r0,                    r18                    \n" \
                "sts  osc+0*%[mul]+%[pha],   r0                     \n" \
                "lds  r18,                   osc+0*%[mul]+%[fre]+1  \n" \
                "lds  r1,                    osc+0*%[mul]+%[pha]+1  \n" \
                "adc  r1,                    r18                    \n" \
                "sts  osc+0*%[mul]+%[pha]+1, r1                     \n" \
                \
                "mov  r18,                   r1                     \n" \
                "lsl  r18                                           \n" \
                "and  r18,                   r1                     \n" \
                "lds  r27,                   osc+0*%[mul]+%[vol]    \n" \
                "sbrc r18,                   7                      \n" \
                "neg  r27                                           \n" \
                "add  r26,                   r27                    \n" \
                \
                "lds  r18,                   osc+1*%[mul]+%[fre]    \n" \
                "lds  r0,                    osc+1*%[mul]+%[pha]    \n" \
                "add  r0,                    r18                    \n" \
                "sts  osc+1*%[mul]+%[pha],   r0                     \n" \
                "lds  r18,                   osc+1*%[mul]+%[fre]+1  \n" \
                "lds  r1,                    osc+1*%[mul]+%[pha]+1  \n" \
                "adc  r1,                    r18                    \n" \
                "sts  osc+1*%[mul]+%[pha]+1, r1                     \n" \
                \
                "lds  r27,                   osc+1*%[mul]+%[vol]    \n" \
                "sbrc r1,                    7                      \n" \
                "neg  r27                                           \n" \
                "add  r26,                   r27                    \n" \
                \
                "ldi  r27,                   1                      \n" \
                "lds  r0,                    osc+3*%[mul]+%[fre]    \n" \
                "lds  r1,                    osc+3*%[mul]+%[fre]+1  \n" \
                "add  r0,                    r0                     \n" \
                "adc  r1,                    r1                     \n" \
                "sbrc r1,                    7                      \n" \
                "eor  r0,                    r27                    \n" \
                "sbrc r1,                    6                      \n" \
                "eor  r0,                    r27                    \n" \
                "sts  osc+3*%[mul]+%[fre],   r0                     \n" \
                "sts  osc+3*%[mul]+%[fre]+1, r1                     \n" \
                \
                "lds  r27,                   osc+3*%[mul]+%[vol]    \n" \
                "sbrc r1,                    7                      \n" \
                "neg  r27                                           \n" \
                "add  r26,                   r27                    \n" \
                \
                "lds  r27,                   pcm                    \n" \
                "add  r26,                   r27                    \n" \
                "sts  %[reg],                r26                    \n" \
                "sts  %[reg2],               r26                    \n" \
                \
                "lds  r27,                   cia_count+1            \n" \
                "lds  r26,                   cia_count              \n" \
                "sbiw r26,                   1                      \n" \
                "breq call_playroutine                              \n" \
                "sts  cia_count+1,           r27                    \n" \
                "sts  cia_count,             r26                    \n" \
                "rjmp 2f                                            \n" \
                \
                "call_playroutine:                                  \n" \
                \
                "lds  r27, cia+1                                    \n" \
                "lds  r26, cia                                      \n" \
                "sts  cia_count+1,           r27                    \n" \
                "sts  cia_count,             r26                    \n" \
                \
                "sei                                                \n" \
                "push r19                                           \n" \
                "push r20                                           \n" \
                "push r21                                           \n" \
                "push r22                                           \n" \
                "push r23                                           \n" \
                "push r24                                           \n" \
                "push r25                                           \n" \
                "push r30                                           \n" \
                "push r31                                           \n" \
                \
                "clr  r1                                            \n" \
                "call ATM_playroutine                               \n" \
                \
                "pop  r31                                           \n" \
                "pop  r30                                           \n" \
                "pop  r25                                           \n" \
                "pop  r24                                           \n" \
                "pop  r23                                           \n" \
                "pop  r22                                           \n" \
                "pop  r21                                           \n" \
                "pop  r20                                           \n" \
                "pop  r19                                           \n" \
                "2:                                                 \n" \
                "pop  r1                                            \n" \
                "pop  r0                                            \n" \
                "pop  r26                                           \n" \
                "pop  r27                                           \n" \
                "3:                                                 \n" \
                "pop  r18                                           \n" \
                "out  __SREG__,              r2                     \n" \
                "pop  r2                                            \n" \
                "reti                                               \n" \
                : \
                : [reg]  "M" _SFR_MEM_ADDR(TARGET_REGISTER), \
                  [reg2] "M" _SFR_MEM_ADDR(TARGET_REGISTER2), \
                  [mul]  "M" (sizeof(Oscillator)), \
                  [pha]  "M" (offsetof(Oscillator, phase)), \
                  [fre]  "M" (offsetof(Oscillator, freq)), \
                  [vol]  "M" (offsetof(Oscillator, vol)) \
              ); \
}
#endif
#endif
