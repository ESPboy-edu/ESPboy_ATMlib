#!/usr/bin/env python3
"""
"Cutlass Cove" - original tropical-pirate showcase for ATMlib (one-shot, 32 bars, ~73 s).

  INTRO  4 bars  fade-in: sea (tremolo noise), seagull (glissando), pad (arp), bass swell
  A      8 bars  sunny calypso/reggae: steel-drum lead, off-beat skank, bouncy bass, one-drop
  B      8 bars  storm / swashbuckle: tempo up, thunder (noise retrigger), cannon (freq slide),
                 staccato lead (note cut), vibrato lead, driving drums, tremolo snare rolls
  A'     8 bars  return, lifted a whole step (C -> D major), tempo back
  OUTRO  4 bars  ritardando (ADD_TEMPO), fade-out on all channels, final cue

Track 0 is only CH3's entry point and is never called (ATMlib starts every channel at 'track 0').
"""
import sys
sys.path.insert(0, '/home/claude/work')
import atmsim
from atmsim import simulate, note_name

SONG_VAR = 'cutlassCove'
TEMPO = 30            # doubled, as always  (30 -> ~112 BPM in the emulator's scale)

_NAMES = ['C', 'C_', 'D', 'D_', 'E', 'F', 'F_', 'G', 'G_', 'A', 'A_', 'B']
_SHARP = {'C': 0, 'C#': 1, 'D': 2, 'D#': 3, 'E': 4, 'F': 5, 'F#': 6, 'G': 7, 'G#': 8,
          'A': 9, 'A#': 10, 'B': 11}


def nnum(name):
    if name[1] == '#':
        pc, octv = name[:2], int(name[2:])
    else:
        pc, octv = name[0], int(name[1:])
    return (octv - 2) * 12 + _SHARP[pc] + 1


def NOTE(name):
    n = nnum(name)
    o = 2 + (n - 1) // 12
    pc = _NAMES[(n - 1) % 12]
    return (f'ATM_NOTE_{pc[0]}{o}{"_" if len(pc) > 1 else ""}', [n])


def DELAY(d):
    assert 1 <= d <= 64, d
    return (f'ATM_DELAY({d})', [0x9F + d])


def _s(v):
    return f'(uint8_t){v}' if v < 0 else str(v)


def VOL(v):         return (f'ATM_VOL({v})', [0x40, v])
def SLV(v):         return (f'ATM_SL_VOL({_s(v)})', [0x41, v & 0xFF])
def SLVADV(a, t):   return (f'ATM_SL_VOL_ADV({_s(a)}, {t})', [0x42, a & 0xFF, t])
SLVOFF = ('ATM_SL_VOL_OFF', [0x43])
def SLFRQ(v):       return (f'ATM_SL_FRQ({_s(v)})', [0x44, v & 0xFF])
def ARP(a, t):      return (f'ATM_ARP(0x{a:02X}, 0x{t:02X})', [0x47, a, t])
def NOISE(v):       return (f'ATM_NOISE(0x{v:02X})', [0x49, v])
NOISEOFF = ('ATM_NOISE_OFF', [0x4A])
def ADDTRA(v):      return (f'ATM_ADD_TRA({_s(v)})', [0x4B, v & 0xFF])
def SETTRA(v):      return (f'ATM_SET_TRA({_s(v)})', [0x4C, v & 0xFF])
def TREM(d, r):     return (f'ATM_TREM({d}, {r})', [0x4E, d, r])
TREMOFF = ('ATM_TREM_OFF', [0x4F])
def VIB(d, r):      return (f'ATM_VIB({d}, {r})', [0x50, d, r])
VIBOFF = ('ATM_VIB_OFF', [0x51])
def GLIS(v):        return (f'ATM_GLIS(0x{v:02X})', [0x52, v])
GLISOFF = ('ATM_GLIS_OFF', [0x53])
def CUT(v):         return (f'ATM_CUT(0x{v:02X})', [0x54, v])
CUTOFF = ('ATM_CUT_OFF', [0x55])
def CUE(v):         return (f'ATM_CUE({v})', [0x57, v])
def ADDTEMPO(v):    return (f'ATM_ADD_TEMPO({_s(v)})', [0x9C, v & 0xFF])
def TEMPOc(v):      return (f'ATM_SET_TEMPO({v})', [0x9D, v])
def STOP():         return ('ATM_STOP_CHAN', [0x9F])
def RET():          return ('ATM_RETURN', [0xFE])
def GOTO(t):        return ('GOTO', t)
def REPEAT(r, t):   return ('REPEAT', r, t)


TR, ORDER = {}, []


def track(name, cmds):
    TR[name] = cmds
    ORDER.append(name)


def n_(name, d):
    return [NOTE(name), DELAY(d)]


# =============================================================================== CH3 drums / sea
# (track 0 = CH3 entry)
track('drums', [
    TEMPOc(TEMPO),
    CUE(1),                                                  # intro
    VOL(4), SLVADV(1, 7), TREM(1, 15),                       # sea: fade-in + slow tremolo swell
    DELAY(64), DELAY(32), SLVOFF,                            # stop the fade at ~tick 96 (slide does NOT clamp at 63!)
    DELAY(32), DELAY(64), DELAY(48),
    GOTO('roll'),                                            # tremolo snare roll into the A section
    CUE(2),                                                  # A
    REPEAT(1, 'dr_a4'),
    CUE(3),                                                  # B
    ADDTEMPO(6),                                             # tempo up
    GOTO('thunder'), GOTO('half_b'),                         # bar 1: thunder + half a bar of drums
    REPEAT(1, 'bar_b'),                                      # bars 2-3
    GOTO('fill_b'),                                          # bar 4
    GOTO('dr_b4'),                                           # bars 5-8
    CUE(4),                                                  # A'
    TEMPOc(TEMPO),
    REPEAT(1, 'dr_a4'),
    CUE(5),                                                  # outro
    GOTO('bar_a'),
    VOL(20), SLVADV(-1, 5), TREM(1, 15),                     # sea again, fading out
    REPEAT(11, 'slow'),                                      # 12 beats, tempo -1 per beat
    CUE(6),                                                  # the end
    STOP(),
])
track('hat',   [VOL(14), SLV(-7), DELAY(8), RET()])
track('ohat',  [VOL(26), SLV(-3), DELAY(8), RET()])
track('thump', [VOL(46), SLV(-6), DELAY(8), RET()])
track('kick',  [VOL(48), SLV(-12), DELAY(8), RET()])
track('snr',   [VOL(44), SLV(-5), DELAY(8), RET()])
# tremolo (depth 6, rate 0 = flips every tick) on a rising volume = snare roll in 8 bytes
track('roll',  [VOL(10), SLV(2), TREM(6, 0), DELAY(16), TREMOFF, RET()])
# noise retrigger: reseeds the LFSR every tick -> low rumble
track('thunder', [NOISE(0x14), VOL(36), SLV(-1), DELAY(32), NOISEOFF, RET()])
track('bar_a',  [REPEAT(3, 'hat'), GOTO('thump'), REPEAT(1, 'hat'), GOTO('ohat'), RET()])
track('fill_a', [REPEAT(3, 'hat'), GOTO('thump'), GOTO('hat'), GOTO('roll'), RET()])
track('dr_a4',  [REPEAT(2, 'bar_a'), GOTO('fill_a'), RET()])
track('half_b', [GOTO('kick'), GOTO('hat'), GOTO('snr'), GOTO('hat'), RET()])
track('bar_b',  [REPEAT(1, 'half_b'), RET()])
track('fill_b', [GOTO('half_b'), GOTO('kick'), GOTO('hat'), GOTO('roll'), RET()])
track('dr_b4',  [REPEAT(2, 'bar_b'), GOTO('fill_b'), RET()])
track('slow',   [ADDTEMPO(-1), DELAY(16), RET()])

# =============================================================================== CH0 lead
track('lead', [
    # ---- INTRO (4 bars): bar 1 silent, bar 2 seagulls, bars 3-4 a pan-flute foreshadowing
    DELAY(64),
    GOTO('gull'),
    VOL(30), SLV(-1), VIB(3, 3),
    SETTRA(-4), GOTO('sing'),                 # F
    ADDTRA(2), GOTO('sing'),                  # G
    VIBOFF,
    # ---- A (8 bars): steel-drum lead, riff transposed per chord
    VOL(40), SLV(-1),
    SETTRA(0), REPEAT(1, 'phr_a'),
    # ---- B (8 bars): 4 bars of staccato pedal riff (note cut), 4 bars vibrato melody
    VOL(36), SLV(-1),
    SETTRA(0), CUT(0x23), REPEAT(3, 'riff_p'), CUTOFF,
    VIB(3, 3),
    SETTRA(5), GOTO('sing'),                  # Dm
    ADDTRA(-5), GOTO('sing'),                 # Am
    ADDTRA(-4), GOTO('sing'),                 # F
    ADDTRA(2), GOTO('sing'),                  # G7
    VIBOFF,
    # ---- A' (8 bars): same, a whole step higher
    VOL(40), SLV(-1),
    SETTRA(2), REPEAT(1, 'phr_a'),
    # ---- OUTRO (4 bars): fading vibrato melody
    VOL(34), SLVADV(-1, 5), VIB(3, 3),
    SETTRA(5), GOTO('sing'),                  # D
    ADDTRA(-7), GOTO('sing'),                 # G
    ADDTRA(7), GOTO('sing'),                  # D
    NOTE('E5'), DELAY(64),                    # held 5th of D
    STOP(),
])
track('gull', [
    VOL(24), SLV(-1),
    NOTE('G6'), GLIS(0x82), DELAY(18), GLISOFF, DELAY(14),
    NOTE('E6'), GLIS(0x82), DELAY(12), GLISOFF, DELAY(20),
    RET(),
])
track('sing', n_('E5', 32) + n_('A5', 16) + n_('B5', 16) + [RET()])
track('riff_a', (n_('E5', 12) + n_('G5', 4) + n_('A5', 8) + n_('G5', 8) +
                 n_('E5', 12) + n_('D5', 4) + n_('C5', 8) + n_('D5', 8) + [RET()]))
track('riff_b', (n_('E5', 12) + n_('G5', 4) + n_('A5', 8) + n_('G5', 8) +
                 n_('E5', 8) + n_('D5', 8) + n_('C5', 16) + [RET()]))
track('phr_a', [GOTO('riff_a'), ADDTRA(5), GOTO('riff_a'), ADDTRA(2), GOTO('riff_a'),
                ADDTRA(-7), GOTO('riff_b'), RET()])
track('riff_p', (n_('A5', 8) + n_('A5', 8) + n_('E5', 8) + n_('A5', 8) +
                 n_('A5', 8) + n_('E5', 8) + n_('A5', 8) + n_('B5', 8) + [RET()]))

# =============================================================================== CH1 chords
track('chords', [
    # ---- INTRO: slow arpeggio pad, fade-in (positive slide, ends at ~28)
    VOL(0), SLVADV(1, 8), ARP(0x43, 0x23),
    NOTE('C4'), DELAY(64), NOTE('C4'), DELAY(64), NOTE('F4'), DELAY(64), NOTE('G4'), DELAY(64),
    # ---- A: reggae skank on beats 2 and 4
    VOL(30), SLV(-6), ARP(0x43, 0x20),
    GOTO('sk_a1'), GOTO('sk_a2'),
    # ---- B: arpeggio stabs every beat
    VOL(28), SLV(-1),
    GOTO('st_b1'), GOTO('st_b2'),
    # ---- A' (lifted)
    VOL(30), SLV(-6), ARP(0x43, 0x20),
    GOTO('sk_a1b'), GOTO('sk_a2b'),
    # ---- OUTRO: slow pad again, fading
    VOL(26), SLVADV(-1, 6), ARP(0x43, 0x23),
    NOTE('D4'), DELAY(64), NOTE('G4'), DELAY(64), NOTE('D4'), DELAY(64), NOTE('A4'), DELAY(64),
    STOP(),
])


def skank(root):
    return [DELAY(16), NOTE(root), DELAY(32), NOTE(root), DELAY(16), RET()]


for nm, r in [('C', 'C4'), ('F', 'F4'), ('G', 'G4'), ('Am', 'A3'), ('D', 'D4'), ('A', 'A4'), ('Bm', 'B3')]:
    track('sk_' + nm, skank(r))
track('sk_a1', [GOTO('sk_C'), GOTO('sk_F'), GOTO('sk_G'), GOTO('sk_C'), RET()])
track('sk_a2', [ARP(0x34, 0x20), GOTO('sk_Am'), ARP(0x43, 0x20),
                GOTO('sk_F'), GOTO('sk_G'), GOTO('sk_C'), RET()])
track('sk_a1b', [GOTO('sk_D'), GOTO('sk_G'), GOTO('sk_A'), GOTO('sk_D'), RET()])
track('sk_a2b', [ARP(0x34, 0x20), GOTO('sk_Bm'), ARP(0x43, 0x20),
                 GOTO('sk_G'), GOTO('sk_A'), GOTO('sk_D'), RET()])
for nm, r in [('Am', 'A3'), ('G', 'G3'), ('F', 'F3'), ('E', 'E3'), ('Dm', 'D4')]:
    track('st_' + nm, [NOTE(r), DELAY(16), RET()])
track('st_b1', [ARP(0x34, 0x20), REPEAT(3, 'st_Am'),
                ARP(0x43, 0x20), REPEAT(3, 'st_G'), REPEAT(3, 'st_F'), REPEAT(3, 'st_E'), RET()])
track('st_b2', [ARP(0x34, 0x20), REPEAT(3, 'st_Dm'), REPEAT(3, 'st_Am'),
                ARP(0x43, 0x20), REPEAT(3, 'st_F'),
                ARP(0x46, 0x20), REPEAT(3, 'st_G'), RET()])          # G7 shell (G B F)

# =============================================================================== CH2 bass
track('bass', [
    # ---- INTRO: swell (positive slide, ends at ~51)
    VOL(0), SLVADV(1, 4),
    NOTE('C3'), DELAY(64), NOTE('C3'), DELAY(64), NOTE('F3'), DELAY(64), NOTE('G3'), DELAY(64),
    # ---- A
    VOL(63), SLV(-5),
    SETTRA(0), GOTO('bass_p1'), GOTO('bass_p2'),
    # ---- B: cannon boom on beat 1, then galloping eighths; transposed per chord
    SLV(-3),
    SETTRA(-3), GOTO('boom'), GOTO('bass_bx'),            # Am (bar 1: boom + 6 notes)
    ADDTRA(-2), GOTO('bass_b'),                           # G
    ADDTRA(-2), GOTO('bass_b'),                           # F
    ADDTRA(-1), GOTO('bass_b'),                           # E
    ADDTRA(10), GOTO('bass_b'),                           # Dm
    ADDTRA(-5), GOTO('bass_b'),                           # Am
    ADDTRA(-4), GOTO('bass_b'),                           # F
    ADDTRA(2), GOTO('bass_b'),                            # G7
    # ---- A' (lifted)
    VOL(63), SLV(-5),
    SETTRA(2), GOTO('bass_p1'), GOTO('bass_p2'),
    # ---- OUTRO: long notes, fading
    VOL(63), SLVADV(-1, 3), SETTRA(0),
    NOTE('D3'), DELAY(64), NOTE('G2'), DELAY(64), NOTE('D3'), DELAY(64), NOTE('D2'), DELAY(64),
    STOP(),
])
track('bass_h', n_('C3', 12) + n_('G3', 4) + n_('C4', 8) + n_('G3', 8) + [RET()])
track('bass_a', [REPEAT(1, 'bass_h'), RET()])
track('bass_f', [GOTO('bass_h')] + n_('C3', 8) + n_('D3', 8) + n_('E3', 8) + n_('G3', 8) + [RET()])
track('bass_p1', [GOTO('bass_a'), ADDTRA(5), GOTO('bass_a'), ADDTRA(2), GOTO('bass_a'),
                  ADDTRA(-7), GOTO('bass_f'), RET()])
track('bass_p2', [ADDTRA(-3), GOTO('bass_a'), ADDTRA(8), GOTO('bass_a'), ADDTRA(2), GOTO('bass_a'),
                  ADDTRA(-7), GOTO('bass_f'), RET()])
track('bass_bx', (n_('G3', 8) + n_('C3', 8) + n_('C4', 8) + n_('G3', 8) + n_('C3', 8) + n_('G3', 8) + [RET()]))
track('bass_b', n_('C3', 8) + n_('C3', 8) + [GOTO('bass_bx'), RET()])
# cannon: a 16-tick downward frequency slide, then back to the volume slide of the pattern
track('boom', [VOL(63), SLFRQ(-24), NOTE('C3'), DELAY(16), SLV(-3), RET()])

CH_ENTRY = ['lead', 'chords', 'bass', 'drums']
assert ORDER[0] == 'drums'


# ---------------------------------------------------------------- assemble
def assemble():
    idx = {name: i for i, name in enumerate(ORDER)}
    blobs = []
    for name in ORDER:
        b = []
        for c in TR[name]:
            if c[0] == 'GOTO':
                b += [0xFC, idx[c[1]]]
            elif c[0] == 'REPEAT':
                b += [0xFD, c[1], idx[c[2]]]
            else:
                b += c[1]
        blobs.append(b)
    offs, pos = [], 0
    for b in blobs:
        offs.append(pos)
        pos += len(b)
    n = len(ORDER)
    song = [n]
    for o in offs:
        song += [o & 0xFF, o >> 8]
    song += [idx[e] for e in CH_ENTRY]
    for b in blobs:
        song += b
    return song, idx, offs, blobs


HEADER = '''#ifndef CUTLASS_H
#define CUTLASS_H

// ---------------------------------------------------------------------------
//  "Cutlass Cove" - an original tropical-pirate showcase for ATMlib       {total} bytes
//
//  Play with:   #include "cutlass.h"
//               ATM.play(cutlassCove);
//
//  A ONE-SHOT: it ends by itself after 2048 ticks (32 bars).  Tempo is set INSIDE the song
//  (ATM_SET_TEMPO({tempo}), then ATM_ADD_TEMPO changes), so do not call ATM.setTempo() while it plays.
//
//  FORM (1 bar = 64 ticks = 4 beats of 16 ticks)
//     bars  1- 4  INTRO   fade-in: sea, seagulls, arpeggio pad, bass swell        CUE 1
//     bars  5-12  A       C | F | G | C   Am | F | G | C   (calypso / reggae)       CUE 2
//     bars 13-20  B       Am G F E   Dm Am F G7  (storm, tempo +6)                CUE 3
//     bars 21-28  A'      same as A, lifted a whole step: D | G | A | D ...       CUE 4
//     bars 29-32  OUTRO   ritardando (tempo -1 per beat) + fade-out               CUE 5
//     last tick                                                                   CUE 6
//  Poll the cues from your sketch with  uint8_t c = ATM.check();  (0 = nothing new)
//
//  WHAT IT SHOWS OFF  (nearly every ATMlib command)
//     ATM_SET_TEMPO / ATM_ADD_TEMPO      A->B speed-up, B->A' reset, outro ritardando (-1 per beat)
//     ATM_CUE                            section markers 1..6 (and a final "song ended" 6)
//     ATM_GOTO / ATM_REPEAT / ATM_RETURN nested tracks: bars are tracks, phrases call bars, depth 4
//     ATM_ADD_TRA / ATM_SET_TRA          ONE riff per chord on lead + bass (C F G C, Am F G C ...),
//                                        the whole A' section is the A tracks + SET_TRA(2)
//     ATM_ARP                            CH1 pad (slow, 4 ticks/step), skank + stabs (1 tick/step),
//                                        major 0x43, minor 0x34 and a G7 shell 0x46
//     ATM_VOL / ATM_SL_VOL               plucked steel-drum lead, skank decay, drum hits
//     ATM_SL_VOL_ADV / ATM_SL_VOL_OFF    long fades: +1 every 5-9 ticks in, -1 every 4-7 ticks out
//     ATM_SL_FRQ                         the cannon "boom" on the bass at the start of B
//     ATM_GLIS                           the two seagull cries (falling glissando)
//     ATM_VIB                            pan-flute / singing lead
//     ATM_CUT                            staccato pedal riff in B (gate: 4 ticks on, 4 off)
//     ATM_TREM                           sea swell (slow) and the snare roll (flip every tick)
//     ATM_NOISE                          thunder rumble at the start of B
//     ATM_STOP_CHAN                      every channel stops by itself at the end
//  Not used here: ATM_GOTO_ADV (loops), ATM_SL_FRQ_ADV, the *_OFF of arp/transpose.
//
//  CHANNELS (ATMlib defaults)
//     CH0 PULSE   lead      CH1 SQUARE  chords      CH2 SAW  bass      CH3 NOISE  drums + sea + cues
//
//  Needs every ATM_FX_* option left ON (the default).  Track 0 is only CH3's entry point and is
//  never called: ATMlib starts every channel with "current track = 0".
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif


'''


def render_text(idx, offs, blobs, total):
    L = [HEADER.format(total=total, tempo=TEMPO)]
    L.append(f'Song {SONG_VAR}[] = {{     // total song bytes = {total}')
    L.append(f'  0x{len(ORDER):02X},                       // Number of tracks')
    for i, name in enumerate(ORDER):
        o = offs[i]
        L.append(f'  0x{o & 0xFF:02X}, 0x{o >> 8:02X},                 // Address of track {i:<2d} {o:5d}   {name}')
    L.append('')
    for k, e in enumerate(CH_ENTRY):
        L.append(f'  0x{idx[e]:02X},                         // CH{k} entry -> track {idx[e]} ({e})')
    L.append('')
    for i, name in enumerate(ORDER):
        L.append(f'  //"Track {i}" {name}  [{len(blobs[i])}b]')
        for c in TR[name]:
            if c[0] == 'GOTO':
                L.append(f'  ATM_GOTO({idx[c[1]]}),   // -> {c[1]}')
            elif c[0] == 'REPEAT':
                L.append(f'  ATM_REPEAT({c[1]}, {idx[c[2]]}),   // {c[1] + 1}x {c[2]}')
            else:
                L.append(f'  {c[0]},')
        L.append('')
    L += ['};', '', '#endif', '']
    return '\n'.join(L)


if __name__ == '__main__':
    song, idx, offs, blobs = assemble()
    total = len(song)
    print(f'tracks={len(ORDER)} total={total} bytes')
    for i, name in enumerate(ORDER):
        print(f'  T{i:<2d} {name:10s} {len(blobs[i]):3d}b', end='   ' if i % 4 != 3 else '\n')
    print()
    open('/home/claude/work/cutlass.bin', 'wb').write(bytes(song))
    open('/home/claude/work/cutlass_text.h', 'w').write(render_text(idx, offs, blobs, total))
    print('wrote cutlass.bin / cutlass_text.h')
