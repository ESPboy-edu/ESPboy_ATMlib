#!/usr/bin/env python3
"""
"Star Light" - original K-pop style dance track for ATMlib: 8-bar intro (once), then an 84-bar body that loops forever.
See HEADER below for the form.  Track 0 is only CH3's entry point and is never called from another track.

LOOPING (the hop-free method, see LearnATMlib.md section 6): every channel has a LOOP track that calls the body
and then calls ITSELF (GOTO to the track it is already in = same-tick restart, no STOP, no silent tick).
The entry track plays the intro once, then calls the loop track and never comes back.
"""
import sys
sys.path.insert(0, '.')
import atmsim
from atmsim import simulate, note_name

SONG_VAR = 'starLight'
TEMPO = 33            # doubled, as always (emulator ~16.5): ~124 BPM

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
def ARP(a, t):      return (f'ATM_ARP(0x{a:02X}, 0x{t:02X})', [0x47, a, t])
def ADDTRA(v):      return (f'ATM_ADD_TRA({_s(v)})', [0x4B, v & 0xFF])
def SETTRA(v):      return (f'ATM_SET_TRA({_s(v)})', [0x4C, v & 0xFF])
def TREM(d, r):     return (f'ATM_TREM({d}, {r})', [0x4E, d, r])
TREMOFF = ('ATM_TREM_OFF', [0x4F])
def VIB(d, r):      return (f'ATM_VIB({d}, {r})', [0x50, d, r])
VIBOFF = ('ATM_VIB_OFF', [0x51])
def GLIS(v):        return (f'ATM_GLIS(0x{v:02X})', [0x52, v])
GLISOFF = ('ATM_GLIS_OFF', [0x53])
def TEMPOc(v):      return (f'ATM_SET_TEMPO({v})', [0x9D, v])
def RET():          return ('ATM_RETURN', [0xFE])
def GOTO(t):        return ('GOTO', t)
def REPEAT(r, t):   return ('REPEAT', r, t)


TR, ORDER = {}, []


def track(name, cmds):
    assert name not in TR, name
    TR[name] = cmds
    ORDER.append(name)


def n_(name, d):
    return [NOTE(name), DELAY(d)]


def seq(*items):
    """items: names -> GOTO, ints -> ADDTRA"""
    out = []
    for it in items:
        out.append(ADDTRA(it) if isinstance(it, int) else GOTO(it))
    return out


# ============================================================ entries (track 0 = CH3 drums entry)
# entry = intro once, then the loop track.  loop track = body, then itself.
track('drums',  [TEMPOc(TEMPO), GOTO('d_intro'), GOTO('drums_loop')])
track('lead',   [GOTO('l_intro'), GOTO('lead_loop')])
track('chords', [GOTO('c_intro'), GOTO('chords_loop')])
track('bass',   [GOTO('b_intro'), GOTO('bass_loop')])
track('lead_loop',   [GOTO('l_body'), GOTO('lead_loop')])
track('chords_loop', [GOTO('c_body'), GOTO('chords_loop')])
track('bass_loop',   [GOTO('b_body'), GOTO('bass_loop')])
track('drums_loop',  [GOTO('d_body'), GOTO('drums_loop')])

# ============================================================ LEAD (CH0 PULSE)
# -- chorus hook, C pentatonic; transposed per chord by the phrase tracks (C +0, G +7, Am +0, F +5)
track('hookA', n_('G5', 8) + n_('G5', 4) + n_('A5', 4) + n_('G5', 8) + n_('E5', 8) + n_('D5', 8) + n_('E5', 8) + n_('G5', 16) + [RET()])
track('hookB', n_('A5', 8) + n_('A5', 4) + n_('C6', 4) + n_('A5', 8) + n_('G5', 8) + n_('E5', 8) + n_('G5', 8) + n_('A5', 16) + [RET()])
track('hookC', n_('C6', 8) + n_('A5', 8) + n_('G5', 8) + n_('E5', 8) + n_('G5', 16) + n_('A5', 16) + [RET()])
track('hookE', n_('E5', 8) + n_('G5', 8) + n_('A5', 8) + n_('G5', 8) + n_('E5', 16) + n_('D5', 16) + [RET()])
track('ch_ph1', seq('hookA', 7, 'hookA', -7, 'hookB', 5, 'hookC', -5) + [RET()])
track('ch_ph2', seq('hookA', 7, 'hookA', -7, 'hookB', 5, 'hookE', -5) + [RET()])
# -- verse (absolute notes in the C pentatonic, lower register)
track('vA', n_('A4', 8) + n_('C5', 4) + n_('A4', 4) + n_('E5', 8) + n_('C5', 8) + n_('A4', 8) + n_('C5', 8) + n_('E5', 8) + n_('D5', 8) + [RET()])
track('vB', n_('C5', 8) + n_('A4', 4) + n_('C5', 4) + n_('E5', 8) + n_('D5', 8) + n_('C5', 8) + n_('A4', 8) + n_('G4', 8) + n_('A4', 8) + [RET()])
track('vC', n_('E5', 8) + n_('G5', 4) + n_('E5', 4) + n_('D5', 8) + n_('C5', 8) + n_('D5', 8) + n_('E5', 8) + n_('G5', 8) + n_('E5', 8) + [RET()])
track('vD', n_('D5', 8) + n_('E5', 4) + n_('D5', 4) + n_('C5', 8) + n_('A4', 8) + n_('G4', 8) + n_('A4', 8) + n_('C5', 8) + n_('D5', 8) + [RET()])
track('v_ph', seq('vA', 'vB', 'vC', 'vD') + [RET()])
# -- pre-chorus: one rising motif on F G F G (major pentatonic from F), +2 for the G bars
track('pm',    n_('F5', 8) + n_('G5', 8) + n_('A5', 8) + n_('C6', 8) + n_('A5', 8) + n_('C6', 8) + n_('D6', 16) + [RET()])
track('pmEnd', n_('F5', 8) + n_('G5', 8) + n_('A5', 8) + n_('C6', 8) + n_('D6', 32) + [RET()])
track('pre_ph', seq('pm', 2, 'pm', -2, 'pm', 2, 'pmEnd', -2) + [RET()])
# -- dance break: 16th riff, root/5th/octave/9th so it survives transposition (Am F C G = 0 -4 +3 -2)
track('cell',  n_('A5', 4) + n_('A5', 4) + n_('E5', 4) + n_('A5', 4) + [RET()])
track('cellB', n_('A5', 4) + n_('E5', 4) + n_('A5', 4) + n_('B5', 4) + [RET()])
track('bar_d', [REPEAT(2, 'cell'), GOTO('cellB'), RET()])
track('dance_ph', seq('bar_d', -4, 'bar_d', 7, 'bar_d', -5, 'bar_d', 2) + [RET()])
# -- bridge: long vibrato notes, then a rising glissando riser
track('br_a', (n_('C6', 32) + n_('A5', 32) + n_('B5', 32) + n_('D6', 32) +
               n_('G5', 32) + n_('B5', 32) + n_('A5', 64) + [RET()]))
track('br_b', (n_('A5', 16) + n_('C6', 16) + n_('F6', 32) + n_('D6', 16) + n_('B5', 16) + n_('G5', 32) +
               n_('B5', 16) + n_('E6', 16) + n_('G6', 32) +
               [NOTE('E5'), GLIS(0x01), DELAY(32), GLISOFF, DELAY(32), RET()]))

track('l_intro', [DELAY(64)] * 4 + [VOL(26), SLV(-1), SETTRA(0), GOTO('ch_ph1'), RET()])
body_lead = [
    SETTRA(0),
    VOL(36), SLV(-1), REPEAT(1, 'v_ph'),                                    # verse 1
    VOL(40), GOTO('pre_ph'),                                                # pre 1
    VOL(46), GOTO('ch_ph1'), GOTO('ch_ph2'),                                # chorus 1
    VOL(40), SLV(-1), GOTO('dance_ph'),                                     # dance break 1
    VOL(36), ADDTRA(12), REPEAT(1, 'v_ph'), ADDTRA(-12),                    # verse 2 (octave up)
    VOL(40), GOTO('pre_ph'),                                                # pre 2
    VOL(46), GOTO('ch_ph1'), GOTO('ch_ph2'),                                # chorus 2
    VOL(40), GOTO('dance_ph'),                                              # dance break 2
    VOL(36), SLV(-1), VIB(3, 3), GOTO('br_a'), GOTO('br_b'), VIBOFF,        # bridge
    VOL(34), REPEAT(1, 'v_ph'),                                             # verse 3
    VOL(40), GOTO('pre_ph'),                                                # pre 3
    VOL(46), ADDTRA(2), REPEAT(1, 'ch_pair'), ADDTRA(-2),                   # last chorus, a whole step higher, 2 x (ph1+ph2)
    RET(),
]
track('ch_pair', [GOTO('ch_ph1'), GOTO('ch_ph2'), RET()])
track('l_body', body_lead)

# ============================================================ CHORDS (CH1 SQUARE)
CH = {'C': ('C4', 0x43), 'G': ('G3', 0x43), 'Am': ('A3', 0x34), 'F': ('F3', 0x43), 'Em': ('E3', 0x34),
      'D': ('D4', 0x43), 'A': ('A3', 0x43), 'Bm': ('B3', 0x34)}


def stab(name):
    r, a = CH[name]
    return [ARP(a, 0x20)] + n_(r, 12) + n_(r, 12) + n_(r, 8) + n_(r, 12) + n_(r, 12) + n_(r, 8) + [RET()]


def pad(name):
    r, a = CH[name]
    return [ARP(a, 0x22), NOTE(r), DELAY(64), RET()]


for nm in ['C', 'G', 'Am', 'F', 'D', 'A', 'Bm']:
    track('st_' + nm, stab(nm))
for nm in ['C', 'G', 'Am', 'F', 'Em']:
    track('pd_' + nm, pad(nm))
track('ph_pd_intro',  seq('pd_C', 'pd_G', 'pd_Am', 'pd_F') + [RET()])
track('ph_pd_verse',  seq('pd_Am', 'pd_F', 'pd_C', 'pd_G') + [RET()])
track('ph_pd_bridge', seq('pd_F', 'pd_G', 'pd_Em', 'pd_Am') + [RET()])
track('ph_st_pre',    seq('st_F', 'st_G', 'st_F', 'st_G') + [RET()])
track('ph_st_chor',   seq('st_C', 'st_G', 'st_Am', 'st_F') + [RET()])
track('ph_st_dance',  seq('st_Am', 'st_F', 'st_C', 'st_G') + [RET()])
track('ph_st_lift',   seq('st_D', 'st_A', 'st_Bm', 'st_G') + [RET()])

track('c_intro', [VOL(0), SLVADV(1, 17), GOTO('ph_pd_intro'), GOTO('ph_pd_intro'), RET()])
track('c_body', [
    VOL(16), SLV(0), REPEAT(1, 'ph_pd_verse'),                          # verse 1
    VOL(24), SLV(-2), GOTO('ph_st_pre'),                                # pre 1
    REPEAT(1, 'ph_st_chor'),                                            # chorus 1
    GOTO('ph_st_dance'),                                                # dance 1
    VOL(16), SLV(0), REPEAT(1, 'ph_pd_verse'),                          # verse 2
    VOL(24), SLV(-2), GOTO('ph_st_pre'),                                # pre 2
    REPEAT(1, 'ph_st_chor'),                                            # chorus 2
    GOTO('ph_st_dance'),                                                # dance 2
    VOL(16), SLV(0), REPEAT(1, 'ph_pd_bridge'),                         # bridge
    REPEAT(1, 'ph_pd_verse'),                                           # verse 3
    VOL(24), SLV(-2), GOTO('ph_st_pre'),                                # pre 3
    REPEAT(3, 'ph_st_lift'),                                            # lifted chorus (16 bars)
    RET(),
])

# ============================================================ BASS (CH2 SAW), transposed per chord
track('bassV', n_('C3', 12) + n_('C3', 4) + n_('G3', 8) + n_('C3', 8) + n_('C3', 12) + n_('C3', 4) + n_('C4', 8) + n_('G3', 8) + [RET()])
track('bassC', n_('C3', 8) + n_('C3', 8) + n_('C4', 8) + n_('C3', 8) + n_('C3', 8) + n_('C3', 8) + n_('G3', 8) + n_('C4', 8) + [RET()])
track('bassB', n_('C3', 32) + n_('G3', 16) + n_('C3', 16) + [RET()])
track('bv_ph',  seq(-3, 'bassV', -4, 'bassV', 7, 'bassV', -5, 'bassV', 5) + [RET()])         # Am F C G
track('bp_ph',  seq(-7, 'bassC', 2, 'bassC', -2, 'bassC', 2, 'bassC', 5) + [RET()])          # F G F G
track('bc_ph',  seq('bassC', -5, 'bassC', 2, 'bassC', -4, 'bassC', 7) + [RET()])             # C G Am F
track('bd_ph',  seq(-3, 'bassC', -4, 'bassC', 7, 'bassC', -5, 'bassC', 5) + [RET()])         # Am F C G
track('bb_ph',  seq(-7, 'bassB', 2, 'bassB', -3, 'bassB', 5, 'bassB', 3) + [RET()])          # F G Em Am

track('b_intro', [DELAY(64)] * 4 + [VOL(63), SLV(-3), SETTRA(0), GOTO('bc_ph'), RET()])
track('b_body', [
    SETTRA(0), VOL(63), SLV(-3),
    REPEAT(1, 'bv_ph'),                                                # verse 1
    GOTO('bp_ph'),                                                     # pre 1
    REPEAT(1, 'bc_ph'),                                                # chorus 1
    GOTO('bd_ph'),                                                     # dance 1
    REPEAT(1, 'bv_ph'),                                                # verse 2
    GOTO('bp_ph'),                                                     # pre 2
    REPEAT(1, 'bc_ph'),                                                # chorus 2
    GOTO('bd_ph'),                                                     # dance 2
    SLV(-1), REPEAT(1, 'bb_ph'),                                       # bridge
    SLV(-3), REPEAT(1, 'bv_ph'),                                       # verse 3
    GOTO('bp_ph'),                                                     # pre 3
    ADDTRA(2), REPEAT(3, 'bc_ph'), ADDTRA(-2),                         # lifted chorus
    RET(),
])

# ============================================================ DRUMS (CH3 NOISE)
track('k4',  [VOL(48), SLV(-12), DELAY(4), RET()])
track('s4',  [VOL(44), SLV(-5), DELAY(4), RET()])
track('h4',  [VOL(14), SLV(-7), DELAY(4), RET()])
track('o4',  [VOL(26), SLV(-3), DELAY(4), RET()])
track('b1',  [GOTO('k4'), REPEAT(2, 'h4'), RET()])                 # K h h h
track('b2',  [GOTO('s4'), REPEAT(1, 'h4'), GOTO('k4'), RET()])     # S h h K
track('b3',  [GOTO('k4'), GOTO('h4'), GOTO('o4'), GOTO('h4'), RET()])   # K h O h
track('b4',  [GOTO('s4'), GOTO('h4'), GOTO('k4'), GOTO('h4'), RET()])   # S h K h
track('bS',  [GOTO('s4'), REPEAT(2, 'h4'), RET()])                 # S h h h
track('bhh', [REPEAT(3, 'h4'), RET()])                             # h h h h
track('bf',  [GOTO('k4'), GOTO('h4'), GOTO('k4'), GOTO('h4'), RET()])   # four on the floor, K h K h
track('roll16', [VOL(10), SLV(2), TREM(6, 0), DELAY(16), TREMOFF, RET()])
track('roll64', [VOL(10), SLVADV(1, 1), TREM(6, 0), DELAY(64), TREMOFF, RET()])
track('bar_g',   [GOTO('b1'), GOTO('b2'), GOTO('b3'), GOTO('b4'), RET()])
track('bar_gf',  [GOTO('b1'), GOTO('b2'), GOTO('b3'), GOTO('roll16'), RET()])
track('bar_v',   [GOTO('b1'), GOTO('bS'), GOTO('b1'), GOTO('bS'), RET()])
track('bar_vf',  [GOTO('b1'), GOTO('bS'), GOTO('b1'), GOTO('roll16'), RET()])
track('bar_hat', [REPEAT(3, 'bhh'), RET()])
track('bar_df',  [REPEAT(3, 'bf'), RET()])
track('bar_dfill', [REPEAT(2, 'bf'), GOTO('roll16'), RET()])
track('bar_br',  [GOTO('b1'), REPEAT(2, 'bhh'), RET()])
track('dr4v', [REPEAT(2, 'bar_v'), GOTO('bar_vf'), RET()])
track('dr4g', [REPEAT(2, 'bar_g'), GOTO('bar_gf'), RET()])
track('dr4d', [REPEAT(2, 'bar_df'), GOTO('bar_dfill'), RET()])
track('dr_pre', [REPEAT(2, 'bar_g'), GOTO('roll64'), RET()])
track('dr_br',  [REPEAT(5, 'bar_br'), GOTO('bar_g'), GOTO('roll64'), RET()])

track('d_intro', [REPEAT(3, 'bar_hat'), REPEAT(2, 'bar_g'), GOTO('bar_gf'), RET()])
track('d_body', [
    REPEAT(1, 'dr4v'),                                    # verse 1
    GOTO('dr_pre'),                                       # pre 1
    REPEAT(1, 'dr4g'),                                    # chorus 1
    GOTO('dr4d'),                                         # dance 1
    REPEAT(1, 'dr4v'),                                    # verse 2
    GOTO('dr_pre'),                                       # pre 2
    REPEAT(1, 'dr4g'),                                    # chorus 2
    GOTO('dr4d'),                                         # dance 2
    GOTO('dr_br'),                                        # bridge
    REPEAT(1, 'dr4v'),                                    # verse 3
    GOTO('dr_pre'),                                       # pre 3
    REPEAT(3, 'dr4g'),                                    # lifted chorus (16 bars)
    RET(),
])


CH_ENTRY = ['lead', 'chords', 'bass', 'drums']
assert ORDER[0] == 'drums'


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
    song = [len(ORDER)]
    for o in offs:
        song += [o & 0xFF, o >> 8]
    song += [idx[e] for e in CH_ENTRY]
    for b in blobs:
        song += b
    return song, idx, offs, blobs


HEADER = '''#ifndef STARLIGHT_H
#define STARLIGHT_H

// ---------------------------------------------------------------------------
//  "Star Light" - an original K-pop style dance track for ATMlib          {total} bytes
//
//  Play with:   #include "starlight.h"      ATM.play(starLight);
//
//  NEVER ENDS: an 8-bar intro plays once, then the 84-bar body (about 2:50) loops forever.
//  The first pass is intro + body = 92 bars = about 3:00.  There is no ending.
//
//  Timing : ATM_SET_TEMPO({tempo}) (doubled) = about 124 BPM.  1 beat = 16 ticks, 1 bar = 64 ticks.
//  Loop   : every channel's entry track plays the intro once and then calls its LOOP track, which
//           calls the body and then calls ITSELF (ATM_GOTO to the track it is already in).  That
//           restarts in the same tick: no STOP, no ATM_GOTO_ADV, no silent tick, no hop.
//           Every channel's body is exactly 5376 ticks (84 bars) of delay.
//           The body sets volume, slide, arpeggio and transposition itself, so every pass is identical.
//
//  FORM                                                            bars
//     INTRO (once)   pad fades in, hook teaser, groove builds, roll   8
//     verse 1        Am F C G x2   (absolute melody, sparse bass)     8
//     pre-chorus 1   F G F G       (one rising motif, transposed)     4
//     chorus 1       C G Am F x2   (hook transposed per chord)        8
//     dance break 1  Am F C G      (16th riff, transposed per chord)  4
//     verse 2        same, melody an octave higher (ADD_TRA 12)       8
//     pre-chorus 2                                                    4
//     chorus 2                                                        8
//     dance break 2                                                   4
//     bridge         F G Em Am x2  (vibrato, slow pad, riser)         8
//     verse 3                                                         8
//     pre-chorus 3                                                    4
//     last chorus    D A Bm G x4   (key change: a whole step up)     16
//                    ends on G, which is chord V of the verse key: it falls back into Am
//
//  Channels (ATMlib defaults):
//     CH0 PULSE  - lead.  The hook, the pre-chorus motif and the dance riff are ONE track each
//                  and are transposed per chord with ATM_ADD_TRA; the key change is SET at the start.
//     CH1 SQUARE - arpeggio chords: slow pad (verse, bridge, intro) and 3+3+2 stabs (pre, chorus,
//                  dance).  Explicit roots (arpeggio + transposition on one channel would transpose twice).
//     CH2 SAW    - bass, transposed per chord, three patterns (sparse, driving, sustained)
//     CH3 NOISE  - dance groove, four-on-the-floor in the dance breaks, tremolo snare-roll fills
//  Note   : track 0 is only CH3's entry point and is never called from another track.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif


'''


def render_text(idx, offs, blobs, total):
    L = [HEADER.format(total=total, tempo=TEMPO), f'Song {SONG_VAR}[] = {{     // total song bytes = {total}',
         f'  0x{len(ORDER):02X},                       // Number of tracks']
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


I_BARS, B_BARS = 8, 84
SECTIONS = [('verse1', 8), ('pre1', 4), ('chorus1', 8), ('dance1', 4), ('verse2', 8), ('pre2', 4), ('chorus2', 8),
            ('dance2', 4), ('bridge', 8), ('verse3', 8), ('pre3', 4), ('last chorus', 16)]
assert sum(b for _, b in SECTIONS) == B_BARS

if __name__ == '__main__':
    song, idx, offs, blobs = assemble()
    total = len(song)
    print(f'tracks={len(ORDER)} total={total} bytes')
    I, B = I_BARS * 64, B_BARS * 64
    ev, restarts, stops, maxstack, tempo = simulate(song, I + B * 3 + 40)
    TL = atmsim.TIMELINE
    ok = True
    print('tempo', tempo, '| max call depth', maxstack)
    if restarts or [s for s in stops if s[0] > 0]:
        print('FAIL: a channel stopped or the song restarted (it must self-loop)'); ok = False
    if maxstack > 7:
        print('FAIL: call stack deeper than 7'); ok = False
    nz = lambda r: [(v, f if v else 0) for v, f in r]       # pitch of a silent channel is irrelevant
    # pass 1 and pass 2 must be identical tick by tick.  (Pass 0 may differ for a few ticks in the chord volume:
    # the intro's SL_VOL_ADV leaves its slide counter running.  Its note events are identical, see below.)
    for t in range(I + 2 * B, I + 3 * B):
        if nz(TL[t]) != nz(TL[t - B]):
            print(f'FAIL: tick {t} differs from tick {t - B}'); ok = False; break

    def body(k):
        a = I + B * k
        return [(e[0] - a,) + e[1:] for e in ev if a <= e[0] < a + B]
    if not (body(0) == body(1) == body(2)):
        print('FAIL: body passes differ (state leaking across the loop)'); ok = False
    else:
        print(f'body pass 0 == pass 1 == pass 2 ({len(body(0))} events each, period {B} ticks)')
    for n in range(3):
        ns = [e[3] for e in ev if e[1] == n and e[2] == 'note' and e[3] > 0]
        print(f'ch{n} note range {min(ns)}..{max(ns)}  ({note_name(min(ns))}..{note_name(max(ns))})')
        if min(ns) < 1 or max(ns) > 63:
            ok = False
    vmax = max(t[n][0] for t in TL for n in range(4))
    print('vol max', vmax)
    if vmax > 63:
        ok = False
    ev0 = body(0)
    bar = 0
    print()
    for nm, nb in SECTIONS:
        rows = []
        for b in range(bar, bar + nb):
            def first(ch):
                es = [e for e in ev0 if e[1] == ch and e[2] == 'note' and e[0] // 64 == b and e[3] > 0]
                return note_name(es[0][3]) if es else '--'
            rows.append(f'{first(0):>3}/{first(1):>3}/{first(2):>3}')
        print(f'{nm:12s} ' + ' '.join(rows))
        bar += nb
    print('(lead / chord-root / bass, first note of each bar)')
    print('\nOK' if ok else '\nPROBLEMS')
    if ok:
        open('./starlight.bin', 'wb').write(bytes(song))
        open('./starlight.h', 'w').write(render_text(idx, offs, blobs, total))
        print('wrote ./starlight.h and ./starlight.bin')
