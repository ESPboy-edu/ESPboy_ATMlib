#!/usr/bin/env python3
"""
"Quest Theme" - chiptune in-game loop for ATMlib, target <= 250 bytes total.

16 bars x 64 ticks = 1024 ticks, A minor, second half lifted a whole step (Bm), pivoting back through Em.
  CH0 PULSE  lead   : one 2-bar riff, TRANSPOSED per chord (SET_TRA / ADD_TRA)
  CH1 SQUARE chords : ARPEGGIO minor triads (explicit roots - arp + transpose on the
                      same channel would transpose twice, see ATMlibimpl.h line ~457)
  CH2 SAW    bass   : root/octave pattern, TRANSPOSED per chord
  CH3 NOISE  drums  : tempo + kick/hat/snare
LOOPING: every channel's entry track ends with GOTO(itself) (a self-loop). There is no STOP
and no GOTO_ADV, so the loop restarts in the same tick (no silent tick = no 'hop') and every
channel sums to exactly 1024 ticks of delay.
Track 0 is never CALLED from another track (ATMlib starts every channel with 'current track'
= 0, calling track 0 breaks the call stack). It is only CH3's entry point; its own final
GOTO(0) is a call to the track it is already in, which is harmless.
"""
import sys
sys.path.insert(0, '.')
import atmsim
from atmsim import simulate, note_name, parse

SONG_VAR = 'questTheme'
TEMPO = 34            # doubled, as always

# ---------------------------------------------------------------- note helpers
_NAMES = ['C', 'C_', 'D', 'D_', 'E', 'F', 'F_', 'G', 'G_', 'A', 'A_', 'B']
_SHARP = {'C': 0, 'C#': 1, 'D': 2, 'D#': 3, 'E': 4, 'F': 5, 'F#': 6, 'G': 7, 'G#': 8,
          'A': 9, 'A#': 10, 'B': 11}


def nnum(name):
    if name[1] == '#':
        pc, octv = name[:2], int(name[2:])
    else:
        pc, octv = name[0], int(name[1:])
    return (octv - 2) * 12 + _SHARP[pc] + 1


def nmacro(n):
    return 'ATM_NOTE_' + _NAMES[(n - 1) % 12] + str(2 + (n - 1) // 12)


def NOTE(name):
    n = nnum(name)
    # sharps are written with the underscore suffix: ATM_NOTE_F5_
    o = 2 + (n - 1) // 12
    pc = _NAMES[(n - 1) % 12]
    return (f'ATM_NOTE_{pc[0]}{o}{"_" if len(pc) > 1 else ""}', [n])


def DELAY(d):
    assert 1 <= d <= 64, d
    return (f'ATM_DELAY({d})', [0x9F + d])


def _s(v):
    return f'(uint8_t){v}' if v < 0 else str(v)


def VOL(v):       return (f'ATM_VOL({v})', [0x40, v])
def SLV(v):       return (f'ATM_SL_VOL({_s(v)})', [0x41, v & 0xFF])
def ARP(a, t):    return (f'ATM_ARP(0x{a:02X}, 0x{t:02X})', [0x47, a, t])
def ADDTRA(v):    return (f'ATM_ADD_TRA({_s(v)})', [0x4B, v & 0xFF])
def SETTRA(v):    return (f'ATM_SET_TRA({_s(v)})', [0x4C, v & 0xFF])
def TEMPOc(v):    return (f'ATM_SET_TEMPO({v})', [0x9D, v])
def STOP():       return ('ATM_STOP_CHAN', [0x9F])
def RET():        return ('ATM_RETURN', [0xFE])
def GOTO(t):      return ('GOTO', t)
def REPEAT(r, t): return ('REPEAT', r, t)


# ---------------------------------------------------------------- the song
TR = {}          # name -> list of commands
ORDER = []


def track(name, cmds):
    TR[name] = cmds
    ORDER.append(name)


def n_(name, d):
    return [NOTE(name), DELAY(d)]


# 0: drums (CH3 entry). tempo + 32 halves, then loop to itself
track('drums', [
    TEMPOc(TEMPO),
    REPEAT(30, 'drum_half'),             # 31 x 32t = 992t
    GOTO('drum_core'), DELAY(8),         # 24t + 8t = 32t  -> 1024t total
    GOTO('drums'),                       # self-loop: same-tick restart
])
# kick, closed hat, open hat, snare, open hat (its delay is added by the caller)
track('drum_core', [
    VOL(48), SLV(-12), DELAY(4),         # kick click
    VOL(14), SLV(-7), DELAY(4),          # closed hat
    VOL(26), SLV(-3), DELAY(8),          # open hat
    VOL(44), SLV(-5), DELAY(8),          # snare
    VOL(26), SLV(-3),                    # open hat, delay supplied by caller
    RET(),
])
track('drum_half', [GOTO('drum_core'), DELAY(8), RET()])

# 3: lead
track('lead', [
    VOL(38), SLV(-2),
    SETTRA(0), GOTO('lead_cycle'),       # bars 1-6   Am Dm Am
    ADDTRA(7), GOTO('lead_riff'), DELAY(16),   # bars 7-8   Em
    SETTRA(2), GOTO('lead_cycle'),       # bars 9-14  Bm Em Bm   (lifted a whole step)
    ADDTRA(5), GOTO('lead_riff'), DELAY(16),   # bars 15-16 Em (pivot back to Am)
    GOTO('lead'),                        # self-loop
])
track('lead_riff', (
    n_('A4', 8) + n_('C5', 8) + n_('E5', 8) + n_('C5', 8) +
    n_('D5', 12) + n_('C5', 4) + n_('A4', 8) + n_('C5', 8) +
    n_('E5', 8) + n_('D5', 8) + n_('C5', 8) + n_('A4', 8) +
    n_('C5', 8) + n_('G4', 8) + [NOTE('A4')] +      # last delay supplied by caller
    [RET()]
))
track('lead_cycle', [
    GOTO('lead_riff'), DELAY(16),
    ADDTRA(5), GOTO('lead_riff'), DELAY(16),
    ADDTRA(-5), GOTO('lead_riff'), DELAY(16),
    RET(),
])

# 6: chords (explicit roots, minor arpeggio)
ROOTS = ['A3', 'D4', 'A3', 'E4',   'B3', 'E4', 'B3', 'E4']
ch = [VOL(30), SLV(-1), ARP(0x34, 0x20)]
for i, r in enumerate(ROOTS):
    last = (i == len(ROOTS) - 1)
    ch += [NOTE(r), DELAY(64), NOTE(r), DELAY(64)]
ch += [GOTO('chords')]                  # self-loop
track('chords', ch)

# 7: bass
track('bass', [
    VOL(63), SLV(-5),
    SETTRA(0), GOTO('bass_cycle'),                               # bars 1-6   Am Dm Am
    ADDTRA(7), REPEAT(1, 'bass_bar'),                            # bars 7-8   Em
    SETTRA(2), GOTO('bass_cycle'),                               # bars 9-14  Bm Em Bm (lifted)
    SETTRA(7), REPEAT(1, 'bass_bar'),                            # bars 15-16 Em
    GOTO('bass'),                                                # self-loop
])
track('bass_cycle', [
    REPEAT(1, 'bass_bar'),
    ADDTRA(5), REPEAT(1, 'bass_bar'),
    ADDTRA(-5), REPEAT(1, 'bass_bar'),
    RET(),
])
track('bass_bar', [REPEAT(2, 'bass_beat'), GOTO('bass_beat4'), RET()])
track('bass_beat', n_('A2', 8) + n_('A3', 8) + [RET()])
track('bass_beat4', n_('A2', 8) + n_('A2', 4) + n_('A3', 4) + [RET()])

CH_ENTRY = ['lead', 'chords', 'bass', 'drums']


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


def render_text(idx, offs, blobs, total):
    L = []
    L += [
        '#ifndef QUEST_H', '#define QUEST_H', '',
        '// ---------------------------------------------------------------------------',
        f'//  "Quest Theme" - chiptune in-game loop, A minor, 16 bars, {total} bytes',
        '//',
        '//  Play with:   #include "quest.h"      ATM.play(questTheme);',
        '//',
        f'//  Timing : ATM_SET_TEMPO({TEMPO}) (doubled).  1 beat = 16 ticks, 1 bar = 64 ticks,',
        '//           16 bars = 1024 ticks. Every channel sums to exactly 1024 ticks of delay and its',
        '//           entry track ends with ATM_GOTO(itself): same-tick restart, no STOP, no hop.',
        '//  Form   : bars 1-8   Am | Am | Dm | Dm | Am | Am | Em | Em',
        '//           bars 9-16  the cycle lifted a whole step: Bm | Bm | Em | Em | Bm | Bm | Em | Em',
        '//                      (Em is chord iv in Bm and chord v in Am: it pivots back into bar 1)',
        '//  Channels (ATMlib defaults):',
        '//     CH0 PULSE  - lead: ONE 2-bar riff, transposed per chord (SET_TRA / ADD_TRA)',
        '//     CH1 SQUARE - minor-triad ARPEGGIO chords (explicit roots, no transpose on this channel)',
        '//     CH2 SAW    - root/octave bass, transposed per chord, beat-4 variation',
        '//     CH3 NOISE  - tempo + kick click, hats, snare',
        '//  Note   : track 0 is only CH3\'s entry point and is never called from another track. ATMlib',
        '//           starts every channel with "current track = 0", so calling track 0 would break the',
        '//           call stack. (CH3\'s own end-of-track ATM_GOTO(0) is fine: it is a call to the track it is in.)',
        '// ---------------------------------------------------------------------------', '',
        '#include <ATMcmds.h>', '',
        '#ifndef Song', '#define Song const uint8_t PROGMEM', '#endif', '', '',
        f'Song {SONG_VAR}[] = {{     // total song bytes = {total}',
        f'  0x{len(ORDER):02X},                       // Number of tracks',
    ]
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
    print(f'tracks={len(ORDER)} total={total} bytes (limit 250)')
    for i, name in enumerate(ORDER):
        print(f'  T{i:<2d} {name:14s} {len(blobs[i]):3d}b')

    # ---- verification with the control-flow simulator
    L = 1024
    ev, restarts, stops, maxstack, tempo = simulate(song, L * 3 + 40)
    tl = [list(t) for t in atmsim.TIMELINE]
    print('tempo set:', tempo, '| max call depth:', maxstack)
    ok = True
    if restarts:
        print('FAIL: the song restarted through STOP/ADV (it must self-loop)'); ok = False
    if [s_ for s_ in stops if s_[0] > 0]:
        print('FAIL: a channel stopped:', stops[:4]); ok = False
    nz = lambda r: [(v, f if v else 0) for v, f in r]      # a silent channel's pitch is irrelevant
    for t in range(L, 3 * L):
        if nz(tl[t]) != nz(tl[t - L]):
            print(f'FAIL: tick {t} differs from tick {t - L}'); ok = False; break
    if maxstack > 7:
        print('FAIL: call stack deeper than 7'); ok = False

    def loop_events(k):
        return [(t - L * k,) + e[1:] for e in ev for t in [e[0]] if L * k <= t < L * (k + 1)]
    l0, l1, l2 = loop_events(0), loop_events(1), loop_events(2)
    if not (l0 == l1 == l2):
        print('FAIL: loops differ (state leaking across the loop)'); ok = False
    else:
        print(f'loop 0 == loop 1 == loop 2  ({len(l0)} note events each, period {L} ticks)')

    # ---- musical summary per channel/bar
    names = ['CH0 lead ', 'CH1 chord', 'CH2 bass ', 'CH3 drum']
    for n in range(3):
        print('\n' + names[n])
        for bar in range(16):
            row = [note_name(e[3]) + f'@{(e[0] % 64):02d}' for e in l0 if e[1] == n and e[0] // 64 == bar]
            print(f'  bar {bar + 1:2d}: ' + ' '.join(row))
    print('\nOK' if ok else '\nPROBLEMS')
    if ok and total <= 250:
        open('./quest.h', 'w').write(render_text(idx, offs, blobs, total))
        open('./quest.bin', 'wb').write(bytes(song))
        print('wrote ./quest.h')
