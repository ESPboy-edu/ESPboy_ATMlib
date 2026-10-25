#include "ATMlib.h"
// Эталонная частота для идеального звучания генераторов
#define SAMPLE_RATE 31250
#define SOUNDPIN D3

uint16_t cia, cia_count;
static bool sigmadeltaflag = 0;

// Переменные состояния
#if ATM_FX_CUE || ATM_FUNC_CUE
uint8_t atmCue;
#endif
#if ATM_FUNC_PAUSE || ATM_FUNC_RESUME || ATM_FUNC_PLAYPAUSE || ATM_FUNC_SFX
uint8_t atmScorePaused;
#endif
#if ATM_FUNC_SFX
uint8_t atmSfxMask;
#endif

// Обновленный ISR: 31250 Гц для чистого звука, делитель для правильного темпа и очень тихие ударные
void IRAM_ATTR sound_speaker_ISR()
{
  int16_t volSum = 0;

  // Канал 0: Пульс
  osc[0].phase += osc[0].freq; 
  int8_t vol0 = osc[0].vol;
  if (osc[0].phase >= 0xC000) vol0 = -vol0;
  volSum += vol0;

  // Канал 1: Квадрат
  osc[1].phase += osc[1].freq; 
  int8_t vol1 = osc[1].vol;
  if (osc[1].phase & 0x8000) vol1 = -vol1;
  volSum += vol1;

  // Канал 2: Пилообразная волна (SAW)
  osc[2].phase += osc[2].freq;
  int8_t phase2 = (osc[2].phase >> 8) - 128;
  int16_t out2 = phase2 * (int8_t)osc[2].vol;
  volSum += (out2 >> 8);

  // Канал 3: Шум (LFSR)
  uint16_t freq = osc[3].freq; 
  freq <<= 1;
  if (freq & 0x8000) freq ^= 1;
  if (freq & 0x4000) freq ^= 1;
  osc[3].freq = freq;
  
  int8_t vol3 = osc[3].vol;
  if (freq & 0x8000) vol3 = -vol3;
  
  // УДАРНЫЕ: Делим громкость шума на 6 (еще на 50% тише от прошлого варианта)
  volSum += (vol3 / 6);

  // ЗАЩИТА ОТ ТРЕСКА (Soft Clamping): предотвращает искажения при переполнении суммы каналов
  int16_t out = volSum + pcm;
  if (out > 255) out = 255;
  else if (out < 0) out = 0;

  sigmaDeltaWrite(0, (uint8_t)out);

  // КОНТРОЛЬ ТЕМПА: обновляем логику трекера каждый ВТОРОЙ сэмпл (15625 Гц)
  // Это возвращает скорость мелодии к оригиналу 1:1, не снижая качество звука
  static bool tempo_tick = false;
  tempo_tick = !tempo_tick;
  
  if (tempo_tick) {
    if (--cia_count == 0) {
      cia_count = cia;
      ATM_playroutine();
    }
  }
}

byte trackCount;
byte tickRate;
const word *trackList;
const byte *trackBase;
uint8_t pcm = 128;

byte ChannelActiveMute = 0b11110000;
osc_t osc[4];

// Таблица частот
#if ATM_OCTAVE_2 && ATM_OCTAVE_3 && ATM_OCTAVE_4 && ATM_OCTAVE_5 && ATM_OCTAVE_6 && ATM_OCTAVE_7
const word noteTable[64] PROGMEM = {
#else
const word noteTable[] PROGMEM = {
#endif
  0,
#if ATM_OCTAVE_2
  262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494,
#endif
#if ATM_OCTAVE_3
  523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932,  988,
#endif
#if ATM_OCTAVE_4
  1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
#endif
#if ATM_OCTAVE_5
  2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
#endif
#if ATM_OCTAVE_6
  4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902,
#endif
#if ATM_OCTAVE_7
  8372, 8870, 9397,
#endif
};

#if ATM_OCTAVE_2 && ATM_OCTAVE_3 && ATM_OCTAVE_4 && ATM_OCTAVE_5 && ATM_OCTAVE_6 && ATM_OCTAVE_7
#define ATM__FREQ(note) pgm_read_word(&noteTable[note])
#else
static inline uint16_t IRAM_ATTR atmNoteFreq(int note) {
  if (note < 1 || note > 63) return 0;
  uint8_t index = 0;
  uint8_t n = (uint8_t)note;
#if ATM_OCTAVE_2
  if (n <= 12) return pgm_read_word(&noteTable[n]);
  index = 12;
#else
  if (n <= 12) return 0;
#endif
  n -= 12;
#if ATM_OCTAVE_3
  if (n <= 12) return pgm_read_word(&noteTable[index + n]);
  index += 12;
#else
  if (n <= 12) return 0;
#endif
  n -= 12;
#if ATM_OCTAVE_4
  if (n <= 12) return pgm_read_word(&noteTable[index + n]);
  index += 12;
#else
  if (n <= 12) return 0;
#endif
  n -= 12;
#if ATM_OCTAVE_5
  if (n <= 12) return pgm_read_word(&noteTable[index + n]);
  index += 12;
#else
  if (n <= 12) return 0;
#endif
  n -= 12;
#if ATM_OCTAVE_6
  if (n <= 12) return pgm_read_word(&noteTable[index + n]);
  index += 12;
#else
  if (n <= 12) return 0;
#endif
  n -= 12;
#if ATM_OCTAVE_7
  if (n <= 3) return pgm_read_word(&noteTable[index + n]);
#endif
  return 0;
}
#define ATM__FREQ(note) atmNoteFreq((note))
#endif

struct ch_t {
  const byte *ptr;
  byte note;

  word stackPointer[ATM_STACK_DEPTH];
  byte stackCounter[ATM_STACK_DEPTH];
  byte stackTrack[ATM_STACK_DEPTH]; 
  
  byte stackIndex;
  byte repeatPoint;
  word delay;
  byte counter;
  byte track;
  word freq;
  byte vol;

#if ATM_FX_VOL_SLIDE || ATM_FX_FREQ_SLIDE
  char volFreSlide;
  byte volFreConfig;
  byte volFreCount;
#endif

#if ATM_FX_ARP || ATM_FX_NOTE_CUT
  byte arpNotes;
  byte arpTiming;
  byte arpCount;
#endif

#if ATM_FX_NOISE
  byte reConfig;
#endif
  byte reCount;

#if ATM_FX_TRANSPOSE
  char transConfig;
#endif

#if ATM_FX_TREMOLO || ATM_FX_VIBRATO
  byte treviDepth;
  byte treviConfig;
  byte treviCount;
#endif

#if ATM_FX_GLISSANDO
  char glisConfig;
  byte glisCount;
#endif
};

ch_t channel[4];

#if ATM_FUNC_SFX
ch_t sfxV;
uint8_t sfxOsc = 0xFF;
#endif

uint16_t IRAM_ATTR read_vle(const byte **pp) {
  word q = 0;
  byte d;
  do {
    q <<= 7;
    d = pgm_read_byte(*pp);
    (*pp)++;
    q |= (d & 0x7F);
  } while (d & 0x80);
  return q;
}

static inline const byte * IRAM_ATTR getTrackPointer(byte track) {
  const byte* tl = (const byte*)trackList;
  word offset = pgm_read_byte(&tl[track * 2]) | (pgm_read_byte(&tl[track * 2 + 1]) << 8);
  return trackBase + offset;
}

void ATMsynth::play(const byte *song) {
  if (sigmadeltaflag) {
    noInterrupts();
    timer1_disable();
    sigmadeltaflag = false;
    sigmaDeltaDisable();
    interrupts();
  }
  cia_count = 1;
  memset(channel, 0, sizeof(channel));
  ChannelActiveMute = 0b11110000;
  
#if ATM_FX_CUE || ATM_FUNC_CUE
  atmCue = 0;
#endif
#if ATM_FUNC_PAUSE || ATM_FUNC_RESUME || ATM_FUNC_PLAYPAUSE || ATM_FUNC_SFX
  atmScorePaused = 0;
#endif
#if ATM_FUNC_SFX
  atmSfxMask = 0;
  sfxOsc = 0xFF;
#endif

  tickRate = 25;
  cia = 15625 / tickRate;

  osc[3].freq = 0x0001; 
  channel[3].freq = 0x0001; 

  trackCount = pgm_read_byte(song++);
  trackList = (word*)song;
  trackBase = (song += (trackCount << 1)) + 4;
  for (unsigned n = 0; n < 4; n++) {
    channel[n].ptr = getTrackPointer(pgm_read_byte(song++));
  }
  
  noInterrupts();
  sigmaDeltaSetup(0, 65000);
  sigmaDeltaAttachPin(SOUNDPIN);
  sigmaDeltaEnable();
  sigmadeltaflag = true;
  timer1_attachInterrupt(sound_speaker_ISR);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(80000000 / SAMPLE_RATE);
  interrupts(); 
}

#if ATM_FUNC_STOP
void ATMsynth::stop() {
  noInterrupts();
  timer1_disable();
  sigmadeltaflag = false;
  sigmaDeltaDisable();
  interrupts();
  memset(channel, 0, sizeof(channel));
  ChannelActiveMute = 0b11110000;
#if ATM_FX_CUE || ATM_FUNC_CUE
  atmCue = 0;
#endif
#if ATM_FUNC_PAUSE || ATM_FUNC_RESUME || ATM_FUNC_PLAYPAUSE || ATM_FUNC_SFX
  atmScorePaused = 0;
#endif
#if ATM_FUNC_SFX
  atmSfxMask = 0;
  sfxOsc = 0xFF;
#endif
}
#endif

bool ATMsynth::isPlay() {
  return(sigmadeltaflag);
}

#if ATM_FUNC_PLAYPAUSE
void ATMsynth::playPause() {
  if (atmScorePaused) resume();
  else pause();
}
#endif

#if ATM_FUNC_PAUSE
void ATMsynth::pause() {
  if (atmScorePaused) return;
  atmScorePaused = 1;
  for (byte n = 0; n < 4; n++) {
#if ATM_FUNC_SFX
    if (!(atmSfxMask & (1 << n))) osc[n].vol = 0;
#else
    osc[n].vol = 0;
#endif
  }
  if(sigmadeltaflag == true && !atmSfxMask){
    noInterrupts();
    timer1_disable();
    sigmadeltaflag = false;
    sigmaDeltaDisable();
    interrupts();
  }
}
#endif

#if ATM_FUNC_RESUME
void ATMsynth::resume() {
  if (!atmScorePaused) return;
  atmScorePaused = 0;
  if(sigmadeltaflag == false){
    noInterrupts();
    timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
    sigmaDeltaEnable();
    sigmadeltaflag = true;
    interrupts();
  }
}
#endif

#if ATM_FUNC_MUTE
void ATMsynth::muteChannel(byte ch) {
  ChannelActiveMute |= (uint8_t)(1 << ch);
}
#endif

#if ATM_FUNC_UNMUTE
void ATMsynth::unMuteChannel(byte ch) {
  ChannelActiveMute &= (uint8_t)(~(1 << ch));
}
#endif

#if ATM_FUNC_TEMPO
void ATMsynth::setTempo(byte t)
{
  if (t == 0) t = 1;
  tickRate = t;
  cia = 15625 / tickRate;
}
#endif

#if ATM_FUNC_SFX
void ATMsynth::playSfx(const byte *track, byte ch) {
  if (ch > 3) return;

  if (sigmadeltaflag == false) {
    tickRate = tickRate ? tickRate : 25;
    cia = 15625 / tickRate;
    cia_count = 1;
    osc[3].freq = 0x0001;
    channel[3].freq = 0x0001;
    atmScorePaused = 1;
    ChannelActiveMute = 0b11110000;
    
    noInterrupts();
    sigmaDeltaSetup(0, 65000);
    sigmaDeltaAttachPin(SOUNDPIN);
    sigmaDeltaEnable();
    sigmadeltaflag = true;
    timer1_attachInterrupt(sound_speaker_ISR);
    timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
    timer1_write(80000000 / SAMPLE_RATE);
    interrupts(); 
  }

  if (sfxOsc < 4 && sfxOsc != ch)
    atmSfxMask &= (uint8_t)(~(1 << sfxOsc));
  memset(&sfxV, 0, sizeof(sfxV));
  sfxV.ptr = track;
  sfxOsc = ch;
  atmSfxMask = (uint8_t)(1 << ch);
  if (ch == 3) osc[3].freq = 0x0001;
}
#endif

void IRAM_ATTR ATM_playroutine() {
  ch_t *ch;

#if ATM_FUNC_SFX
  for (byte pass = 0; pass < 2; pass++)
#else
  for (byte pass = 0; pass < 1; pass++)
#endif
  for (byte n = 0; n < 4; n++)
  {
    byte out = 1;
    if (pass == 0) {
      ch = &channel[n];
#if ATM_FUNC_PAUSE || ATM_FUNC_RESUME || ATM_FUNC_PLAYPAUSE || ATM_FUNC_SFX
      if (atmScorePaused) {
#if ATM_FUNC_SFX
        if (!(atmSfxMask & (1 << n))) osc[n].vol = 0;
#else
        osc[n].vol = 0;
#endif
        continue;
      }
#endif
#if ATM_FUNC_SFX
      out = !(atmSfxMask & (1 << n));
#endif
    } else {
#if ATM_FUNC_SFX
      if (sfxOsc != n) continue;
      ch = &sfxV;
#else
      continue;
#endif
    }

#if ATM_FX_NOISE
    if (ch->reConfig) {
      if (ch->reCount >= (ch->reConfig & 0x03)) {
        osc[3].freq = ATM__FREQ(ch->reConfig >> 2);
        ch->reCount = 0;
      }
      else ch->reCount++;
    }
#endif

#if ATM_FX_GLISSANDO
    if (ch->glisConfig) {
      if (ch->glisCount >= (ch->glisConfig & 0x7F)) {
        if (ch->glisConfig & 0x80) ch->note -= 1;
        else ch->note += 1;
        if (ch->note < 1) ch->note = 1;
        else if (ch->note > 63) ch->note = 63;
        ch->freq = ATM__FREQ(ch->note);
        ch->glisCount = 0;
      }
      else ch->glisCount++;
    }
#endif

#if ATM_FX_VOL_SLIDE || ATM_FX_FREQ_SLIDE
    if (ch->volFreSlide) {
      if (!ch->volFreCount) {
        int16_t vf = ((ch->volFreConfig & 0x40) ? ch->freq : ch->vol);
        vf += (ch->volFreSlide);
        if (!(ch->volFreConfig & 0x80)) {
          if (vf < 0) vf = 0;
          else if (ch->volFreConfig & 0x40) if (vf > 9397) vf = 9397;
            else if (!(ch->volFreConfig & 0x40)) if (vf > 63) vf = 63;
        }
        (ch->volFreConfig & 0x40) ? ch->freq = vf : ch->vol = vf;
      }
      if (ch->volFreCount++ >= (ch->volFreConfig & 0x3F)) ch->volFreCount = 0;
    }
#endif

#if ATM_FX_ARP || ATM_FX_NOTE_CUT
    if (ch->arpNotes && ch->note) {
      if ((ch->arpCount & 0x1F) < (ch->arpTiming & 0x1F)) ch->arpCount++;
      else {
        if ((ch->arpCount & 0xE0) == 0x00) ch->arpCount = 0x20;
        else if ((ch->arpCount & 0xE0) == 0x20 && !(ch->arpTiming & 0x40) && (ch->arpNotes != 0xFF)) ch->arpCount = 0x40;
        else ch->arpCount = 0x00;
        byte arpNote = ch->note;
        if ((ch->arpCount & 0xE0) != 0x00) {
          if (ch->arpNotes == 0xFF) arpNote = 0;
          else arpNote += (ch->arpNotes >> 4);
        }
        if ((ch->arpCount & 0xE0) == 0x40) arpNote += (ch->arpNotes & 0x0F);
#if ATM_FX_TRANSPOSE
        ch->freq = ATM__FREQ(arpNote + ch->transConfig);
#else
        ch->freq = ATM__FREQ(arpNote);
#endif
      }
    }
#endif

#if ATM_FX_TREMOLO || ATM_FX_VIBRATO
    if (ch->treviDepth) {
      int16_t vt = ((ch->treviConfig & 0x40) ? ch->freq : ch->vol);
      vt = (ch->treviCount & 0x80) ? (vt + ch->treviDepth) : (vt - ch->treviDepth);
      if (vt < 0) vt = 0;
      else if (ch->treviConfig & 0x40) if (vt > 9397) vt = 9397;
        else if (!(ch->treviConfig & 0x40)) if (vt > 63) vt = 63;
      (ch->treviConfig & 0x40) ? ch->freq = vt : ch->vol = vt;
      if ((ch->treviCount & 0x1F) < (ch->treviConfig & 0x1F)) ch->treviCount++;
      else {
        if (ch->treviCount & 0x80) ch->treviCount = 0;
        else ch->treviCount = 0x80;
      }
    }
#endif

    if (ch->delay) {
      if (ch->delay != 0xFFFF) ch->delay--;
    }
    else {
      do {
        byte cmd = pgm_read_byte(ch->ptr++);
        if (cmd < 64) {
#if ATM_FX_TRANSPOSE
          if (ch->note = cmd) ch->note += ch->transConfig;
#else
          ch->note = cmd;
#endif
          ch->freq = ATM__FREQ(ch->note);
#if ATM_FX_VOL_SLIDE || ATM_FX_FREQ_SLIDE
          if (!ch->volFreConfig) ch->vol = ch->reCount;
#else
          ch->vol = ch->reCount;
#endif
#if ATM_FX_ARP || ATM_FX_NOTE_CUT
          if (ch->arpTiming & 0x20) ch->arpCount = 0;
#endif
        }
        else if (cmd < 160) {
          switch (cmd - 64) {
#if ATM_FX_VOLUME
            case 0:
              ch->vol = pgm_read_byte(ch->ptr++);
              ch->reCount = ch->vol;
              break;
#endif
#if ATM_FX_VOL_SLIDE && ATM_FX_FREQ_SLIDE
            case 1: case 4:
              ch->volFreSlide = pgm_read_byte(ch->ptr++);
              ch->volFreConfig = (cmd - 64) == 1 ? 0x00 : 0x40;
              break;
            case 2: case 5:
              ch->volFreSlide = pgm_read_byte(ch->ptr++);
              ch->volFreConfig = pgm_read_byte(ch->ptr++);
              if ((cmd - 64) == 5) ch->volFreConfig |= 0x40;
              break;
            case 3: case 6:
              ch->volFreSlide = 0;
              break;
#elif ATM_FX_VOL_SLIDE
            case 1:
              ch->volFreSlide = pgm_read_byte(ch->ptr++);
              ch->volFreConfig = 0x00;
              break;
            case 2:
              ch->volFreSlide = pgm_read_byte(ch->ptr++);
              ch->volFreConfig = pgm_read_byte(ch->ptr++);
              break;
            case 3:
              ch->volFreSlide = 0;
              break;
#elif ATM_FX_FREQ_SLIDE
            case 4:
              ch->volFreSlide = pgm_read_byte(ch->ptr++);
              ch->volFreConfig = 0x40;
              break;
            case 5:
              ch->volFreSlide = pgm_read_byte(ch->ptr++);
              ch->volFreConfig = pgm_read_byte(ch->ptr++);
              ch->volFreConfig |= 0x40;
              break;
            case 6:
              ch->volFreSlide = 0;
              break;
#endif
#if ATM_FX_ARP
            case 7:
              ch->arpNotes = pgm_read_byte(ch->ptr++);
              ch->arpTiming = pgm_read_byte(ch->ptr++);
              break;
            case 8:
              ch->arpNotes = 0;
              break;
#endif
#if ATM_FX_NOISE
            case 9:
              ch->reConfig = pgm_read_byte(ch->ptr++);
              break;
            case 10:
              ch->reConfig = 0;
              break;
#endif
#if ATM_FX_TRANSPOSE
            case 11:
              ch->transConfig += (char)pgm_read_byte(ch->ptr++);
              break;
            case 12:
              ch->transConfig = pgm_read_byte(ch->ptr++);
              break;
            case 13:
              ch->transConfig = 0;
              break;
#endif
#if ATM_FX_TREMOLO && ATM_FX_VIBRATO
            case 14: case 16:
              ch->treviDepth = pgm_read_byte(ch->ptr++);
              ch->treviConfig = pgm_read_byte(ch->ptr++) + ((cmd - 64) == 14 ? 0x00 : 0x40);
              break;
            case 15: case 17:
              ch->treviDepth = 0;
              break;
#elif ATM_FX_TREMOLO
            case 14:
              ch->treviDepth = pgm_read_byte(ch->ptr++);
              ch->treviConfig = pgm_read_byte(ch->ptr++);
              break;
            case 15:
              ch->treviDepth = 0;
              break;
#elif ATM_FX_VIBRATO
            case 16:
              ch->treviDepth = pgm_read_byte(ch->ptr++);
              ch->treviConfig = pgm_read_byte(ch->ptr++) + 0x40;
              break;
            case 17:
              ch->treviDepth = 0;
              break;
#endif
#if ATM_FX_GLISSANDO
            case 18:
              ch->glisConfig = pgm_read_byte(ch->ptr++);
              break;
            case 19:
              ch->glisConfig = 0;
              break;
#endif
#if ATM_FX_NOTE_CUT
            case 20:
              ch->arpNotes = 0xFF;
              ch->arpTiming = pgm_read_byte(ch->ptr++);
              break;
            case 21:
              ch->arpNotes = 0;
              break;
#endif
#if ATM_FX_CUE
            case 23:
              atmCue = pgm_read_byte(ch->ptr++);
              break;
#endif
#if ATM_FX_TEMPO
            case 92:
              tickRate += pgm_read_byte(ch->ptr++);
              cia = 15625 / tickRate;
              break;
            case 93:
              tickRate = pgm_read_byte(ch->ptr++);
              cia = 15625 / tickRate;
              break;
#endif
#if ATM_FX_GOTO_ADV
            case 94:
              for (byte i = 0; i < 4; i++) channel[i].repeatPoint = pgm_read_byte(ch->ptr++);
              break;
#endif
#if ATM_FX_STOP
            case 95:
              if (pass) {
                ch->vol = 0;
                ch->delay = 0xFFFF;
#if ATM_FUNC_SFX
                sfxOsc = 0xFF;
                atmSfxMask &= (uint8_t)(~(1 << n));
#endif
              } else {
                ChannelActiveMute ^= (uint8_t)(1 << (n + 4));
                ch->vol = 0;
                ch->delay = 0xFFFF;
              }
              break;
#endif
          }
        } else if (cmd < 224) {
          ch->delay = cmd - 159;
        } else if (cmd == 224) {
          ch->delay = read_vle(&ch->ptr) + 65;
        } else if (cmd < 252) {
        } else if (cmd == 252 || cmd == 253) {
          byte new_counter = cmd == 252 ? 0 : pgm_read_byte(ch->ptr++);
          byte new_track = pgm_read_byte(ch->ptr++);

          if (new_track != ch->track) {
            ch->stackCounter[ch->stackIndex] = ch->counter;
            ch->stackTrack[ch->stackIndex] = ch->track;
            ch->stackPointer[ch->stackIndex] = ch->ptr - trackBase;
            ch->stackIndex++;
            ch->track = new_track;
          }

          ch->counter = new_counter;
          ch->ptr = getTrackPointer(ch->track);
        } else if (cmd == 254) {
          if (ch->counter > 0 || ch->stackIndex == 0) {
            if (ch->counter) ch->counter--;
            ch->ptr = getTrackPointer(ch->track);
          } else {
            if (ch->stackIndex == 0) {
              ch->delay = 0xFFFF;
            } else {
              ch->stackIndex--;
              ch->ptr = ch->stackPointer[ch->stackIndex] + trackBase;
              ch->counter = ch->stackCounter[ch->stackIndex];
              ch->track = ch->stackTrack[ch->stackIndex];
            }
          }
        }
      } while (ch->delay == 0);

      if (ch->delay != 0xFFFF) ch->delay--;
    }

    if (out && !(ChannelActiveMute & (1 << n))) {
      if (n == 3) {
        osc[n].vol = ch->vol >> 1;
      } else
      {
        osc[n].freq = ch->freq;
        osc[n].vol = ch->vol;
      }
    }

    if (pass == 0 && !(ChannelActiveMute & 0xF0))
    {
      byte repeatSong = 0;
      for (byte j = 0; j < 4; j++) repeatSong += channel[j].repeatPoint;
      if (repeatSong) {
        for (byte k = 0; k < 4; k++) {
          channel[k].ptr = getTrackPointer(channel[k].repeatPoint);
          channel[k].delay = 0;
        }
        ChannelActiveMute = 0b11110000;
      }
#if ATM_FUNC_SFX
      else if (sfxOsc >= 4)
#else
      else
#endif
      {
        ATMsynth::stop();
      }
    }
  }
}

#if ATM_FUNC_CUE
uint8_t ATMsynth::check() {
  uint8_t v = atmCue;
  atmCue = 0;
  return v;
}

uint8_t ATMsynth::check(uint8_t id) {
  return atmCue == id;
}
#endif