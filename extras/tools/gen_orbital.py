import sys
sys.path.insert(0, '.')
import atmsim
from atmsim import simulate, note_name

SONG_VAR = 'orbitalRush'
TEMPO = 37            # doubled, as always: ~139 BPM (16 ticks per beat)

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

# entries: track 0 = CH3 drums (never called from another track); loop tracks call body then themselves
track('drums',  [TEMPOc(TEMPO), GOTO('d_intro'), GOTO('drums_loop')])
track('lead',   [GOTO('l_intro'), GOTO('lead_loop')])
track('chords', [GOTO('c_intro'), GOTO('chords_loop')])
track('bass',   [GOTO('b_intro'), GOTO('bass_loop')])
track('lead_loop',   [GOTO('l_body'), GOTO('lead_loop')])
track('chords_loop', [GOTO('c_body'), GOTO('chords_loop')])
track('bass_loop',   [GOTO('b_body'), GOTO('bass_loop')])
track('drums_loop',  [GOTO('d_body'), GOTO('drums_loop')])

def bars(*notes):
    out = []
    for nm, d in notes:
        out += n_(nm, d)
    return out

# ============================================================ LEAD (CH0 PULSE), D minor, absolute notes
# A section (Dm Bb F C)
track('A1', bars(('A5',16),('F5',8),('A5',8),('D6',24),('C6',8),
                 ('A#5',16),('A5',8),('G5',8),('F5',16),('G5',8),('A5',8),
                 ('A5',8),('C6',8),('F6',24),('E6',8),('C6',8),('A5',8),
                 ('G5',8),('E5',8),('G5',8),('C6',16),('A5',8),('G5',16)) + [RET()])
track('A2', bars(('D6',16),('C6',8),('A5',8),('C6',16),('D6',8),('E6',8),
                 ('D6',16),('C6',8),('A#5',8),('A5',16),('A#5',8),('C6',8),
                 ('A5',8),('A5',8),('C6',8),('F6',16),('E6',8),('D6',8),('C6',8),
                 ('E6',16),('D6',8),('C6',8),('G5',16),('A5',16)) + [RET()])
# B section (Gm Dm Bb C)
track('B1', bars(('G5',16),('A#5',16),('D6',16),('A#5',16),
                 ('A5',16),('D6',16),('F6',16),('D6',16),
                 ('D6',8),('D6',8),('C6',8),('A#5',8),('A5',16),('A#5',16),
                 ('C6',16),('E6',16),('G6',16),('E6',8),('C6',8)) + [RET()])
track('B2', bars(('D6',8),('C6',8),('A#5',8),('G5',8),('A#5',16),('D6',16),
                 ('F6',8),('E6',8),('D6',8),('A5',8),('D6',16),('F6',16),
                 ('D6',16),('C6',8),('A#5',8),('F6',16),('D6',16),
                 ('E6',8),('G6',8),('E6',8),('C6',8),('G5',16),('C6',16)) + [RET()])
# break (Bb C Dm A): long vibrato notes + glissando riser
track('K1', bars(('D6',48),('C6',16),('E6',48),('G6',16),('F6',32),('E6',16),('D6',16),('E6',64)) + [RET()])
track('K2', bars(('D6',32),('F6',32),('E6',32),('G6',32),('A6',32),('F6',32)) +
            [NOTE('C4'), GLIS(0x01), DELAY(64), GLISOFF, RET()])
# chorus (Dm Bb Gm A)
track('C1', bars(('D6',24),('C6',8),('A5',8),('D6',8),('F6',16),
                 ('D6',24),('C6',8),('A#5',8),('C6',8),('D6',16),
                 ('A#5',16),('D6',16),('G6',16),('F6',8),('D6',8),
                 ('E6',8),('C#6',8),('A5',8),('C#6',8),('E6',16),('A5',16)) + [RET()])
track('C2', bars(('F6',24),('E6',8),('D6',8),('A5',8),('D6',16),
                 ('F6',24),('D6',8),('A#5',8),('D6',8),('F6',16),
                 ('G6',16),('F6',16),('D6',16),('A#5',16),
                 ('E6',16),('C#6',16),('A5',16),('A5',8),('C#6',8)) + [RET()])
# dance riff (Dm C Bb A): one-bar riff transposed per chord (0, -2, -4, -5); root/5th/octave only
track('cell',  n_('D6',4) + n_('D6',4) + n_('A5',4) + n_('D6',4) + [RET()])
track('cellB', n_('D6',4) + n_('A5',4) + n_('D6',4) + n_('E6',4) + [RET()])
track('bar_d', [REPEAT(2, 'cell'), GOTO('cellB'), RET()])
track('cell2',  n_('D5',4) + n_('A5',4) + n_('D6',4) + n_('A5',4) + [RET()])
track('cell2b', n_('D6',4) + n_('E6',4) + n_('D6',4) + n_('A5',4) + [RET()])
track('bar_d2', [GOTO('cell2'), GOTO('cell2b'), GOTO('cell2'), GOTO('cell2b'), RET()])
track('dance_ph',  seq('bar_d', -2, 'bar_d', -2, 'bar_d', -1, 'bar_d', 5) + [RET()])
track('dance2_ph', seq('bar_d2', -2, 'bar_d2', -2, 'bar_d2', -1, 'bar_d2', 5) + [RET()])
# tag (Bb Gm C A) leads back to the verse
track('T1', bars(('D6',16),('C6',16),('A#5',32),
                 ('D6',16),('A#5',16),('G5',32),
                 ('E6',16),('C6',16),('G5',32),
                 ('E6',16),('C#6',16),('A5',32)) + [RET()])
track('T2', bars(('D6',8),('D6',8),('C6',8),('A#5',8),('G5',32),
                 ('D6',8),('D6',8),('A#5',8),('G5',8),('D5',32),
                 ('E6',8),('E6',8),('D6',8),('C6',8),('G5',32),
                 ('E6',8),('C#6',8),('A5',8),('E5',8),('A5',32)) + [RET()])
track('A_pair',  [GOTO('A1'), GOTO('A2'), RET()])
track('B_pair',  [GOTO('B1'), GOTO('B2'), RET()])
track('C_pair',  [GOTO('C1'), GOTO('C2'), RET()])
track('l_intro', [DELAY(64)] * 4 + [VOL(26), SLV(-1), SETTRA(0), GOTO('A1'), RET()])
track('l_body', [
    SETTRA(0),
    VOL(34), SLV(-1), REPEAT(1, 'A_pair'),                                  # A   16
    VOL(40), SLV(-1), REPEAT(1, 'B_pair'),                                  # B   16
    VOL(36), SLV(-1), VIB(3, 3), GOTO('K1'), GOTO('K2'), VIBOFF,            # break 8
    VOL(46), SLV(-1), REPEAT(1, 'C_pair'),                                  # chorus 16
    VOL(40), SLV(-1), REPEAT(1, 'dance_ph'), REPEAT(1, 'dance2_ph'),        # dance 16 (2x dance_ph, 2x dance2_ph)
    VOL(46), SLV(-1), ADDTRA(2), REPEAT(1, 'C_pair'), ADDTRA(-2),           # lifted chorus 16 (+2 = Em C Am B)
    VOL(40), SLV(-1), GOTO('T1'), GOTO('T2'),                               # tag 8
    RET(),
])

# ============================================================ CHORDS (CH1 SQUARE), explicit roots
CH = {'Dm': ('D4', 0x34), 'Bb': ('A#3', 0x43), 'F': ('F4', 0x43), 'C': ('C4', 0x43), 'Gm': ('G3', 0x34),
      'A': ('A3', 0x43), 'Em': ('E4', 0x34), 'Am': ('A3', 0x34), 'B': ('B3', 0x43)}

def stab(name):
    r, a = CH[name]
    return [ARP(a, 0x20)] + n_(r, 12) + n_(r, 12) + n_(r, 8) + n_(r, 12) + n_(r, 12) + n_(r, 8) + [RET()]

def pad(name):
    r, a = CH[name]
    return [ARP(a, 0x22), NOTE(r), DELAY(64), RET()]

for nm in ['Gm', 'Dm', 'Bb', 'C', 'A', 'Em', 'Am', 'B']:
    track('st_' + nm, stab(nm))
for nm in ['Dm', 'Bb', 'F', 'C', 'A']:
    track('pd_' + nm, pad(nm))
track('ph_pd_A',  seq('pd_Dm', 'pd_Bb', 'pd_F', 'pd_C') + [RET()])
track('ph_pd_br', seq('pd_Bb', 'pd_C', 'pd_Dm', 'pd_A') + [RET()])
track('ph_st_B',  seq('st_Gm', 'st_Dm', 'st_Bb', 'st_C') + [RET()])
track('ph_st_C',  seq('st_Dm', 'st_Bb', 'st_Gm', 'st_A') + [RET()])
track('ph_st_D',  seq('st_Dm', 'st_C', 'st_Bb', 'st_A') + [RET()])
track('ph_st_L',  seq('st_Em', 'st_C', 'st_Am', 'st_B') + [RET()])
track('ph_st_T',  seq('st_Bb', 'st_Gm', 'st_C', 'st_A') + [RET()])
track('c_intro', [VOL(0), SLVADV(1, 17), GOTO('ph_pd_A'), GOTO('ph_pd_A'), RET()])
track('c_body', [
    VOL(16), SLV(0), REPEAT(3, 'ph_pd_A'),                  # A 16
    VOL(24), SLV(-2), REPEAT(3, 'ph_st_B'),                 # B 16
    VOL(16), SLV(0), REPEAT(1, 'ph_pd_br'),                 # break 8
    VOL(24), SLV(-2), REPEAT(3, 'ph_st_C'),                 # chorus 16
    REPEAT(3, 'ph_st_D'),                                   # dance 16
    REPEAT(3, 'ph_st_L'),                                   # lift 16
    GOTO('ph_st_T'), GOTO('ph_st_T'),                       # tag 8
    RET(),
])

# ============================================================ BASS (CH2 SAW), base D3, transposed per chord
track('bassS', n_('D3', 12) + n_('D3', 4) + n_('A3', 8) + n_('D3', 8) + n_('D3', 12) + n_('D3', 4) + n_('D4', 8) + n_('A3', 8) + [RET()])
track('bassP', n_('D3', 4) + n_('D3', 4) + n_('D4', 4) + n_('D3', 4) + n_('D3', 4) + n_('D3', 4) + n_('D4', 4) + n_('A3', 4) +
               n_('D3', 4) + n_('D3', 4) + n_('D4', 4) + n_('D3', 4) + n_('D3', 4) + n_('D4', 4) + n_('D3', 4) + n_('A3', 4) + [RET()])
track('bassB', n_('D3', 32) + n_('A3', 16) + n_('D3', 16) + [RET()])
track('bAs', seq('bassS', -4, 'bassS', -5, 'bassS', 7, 'bassS', 2) + [RET()])    # Dm Bb F C sparse
track('bAp', seq('bassP', -4, 'bassP', -5, 'bassP', 7, 'bassP', 2) + [RET()])    # Dm Bb F C driving
track('bB',  seq(-7, 'bassP', 7, 'bassP', -4, 'bassP', 2, 'bassP', 2) + [RET()]) # Gm Dm Bb C
track('bC',  seq('bassP', -4, 'bassP', -3, 'bassP', 2, 'bassP', 5) + [RET()])    # Dm Bb Gm A
track('bD',  seq('bassP', -2, 'bassP', -2, 'bassP', -1, 'bassP', 5) + [RET()])   # Dm C Bb A
track('bK',  seq(-4, 'bassB', 2, 'bassB', 2, 'bassB', -5, 'bassB', 5) + [RET()]) # Bb C Dm A
track('bT',  seq(-4, 'bassP', -3, 'bassP', 5, 'bassP', -3, 'bassP', 5) + [RET()])# Bb Gm C A
track('b_intro', [DELAY(64)] * 4 + [VOL(63), SLV(-3), SETTRA(0), GOTO('bAp'), RET()])
track('b_body', [
    SETTRA(0), VOL(63), SLV(-3),
    REPEAT(1, 'bAs'), REPEAT(1, 'bAp'),                     # A 16 (8 sparse, 8 driving)
    REPEAT(3, 'bB'),                                        # B 16
    SLV(-1), REPEAT(1, 'bK'), SLV(-3),                      # break 8
    REPEAT(3, 'bC'),                                        # chorus 16
    REPEAT(3, 'bD'),                                        # dance 16
    ADDTRA(2), REPEAT(3, 'bC'), ADDTRA(-2),                 # lift 16 (+2 = E C A B)
    GOTO('bT'), GOTO('bT'),                                 # tag 8
    RET(),
])

# ============================================================ DRUMS (CH3 NOISE)
track('k4',  [VOL(48), SLV(-12), DELAY(4), RET()])
track('s4',  [VOL(44), SLV(-5), DELAY(4), RET()])
track('h4',  [VOL(14), SLV(-7), DELAY(4), RET()])
track('o4',  [VOL(26), SLV(-3), DELAY(4), RET()])
track('b1',  [GOTO('k4'), REPEAT(2, 'h4'), RET()])
track('b2',  [GOTO('s4'), REPEAT(1, 'h4'), GOTO('k4'), RET()])
track('b3',  [GOTO('k4'), GOTO('h4'), GOTO('o4'), GOTO('h4'), RET()])
track('b4',  [GOTO('s4'), GOTO('h4'), GOTO('k4'), GOTO('h4'), RET()])
track('bS',  [GOTO('s4'), REPEAT(2, 'h4'), RET()])
track('bhh', [REPEAT(3, 'h4'), RET()])
track('bf',  [GOTO('k4'), GOTO('h4'), GOTO('k4'), GOTO('h4'), RET()])
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
track('dr_br', [REPEAT(5, 'bar_br'), GOTO('bar_g'), GOTO('roll64'), RET()])
track('d_intro', [REPEAT(3, 'bar_hat'), REPEAT(2, 'bar_g'), GOTO('bar_gf'), RET()])
track('d_body', [
    REPEAT(3, 'dr4v'),                                      # A 16
    REPEAT(3, 'dr4g'),                                      # B 16
    GOTO('dr_br'),                                          # break 8
    REPEAT(3, 'dr4g'),                                      # chorus 16
    REPEAT(3, 'dr4d'),                                      # dance 16
    REPEAT(3, 'dr4g'),                                      # lift 16
    REPEAT(1, 'dr4g'),                                      # tag 8
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
            if c[0] == 'GOTO': b += [0xFC, idx[c[1]]]
            elif c[0] == 'REPEAT': b += [0xFD, c[1], idx[c[2]]]
            else: b += c[1]
        blobs.append(b)
    offs, pos = [], 0
    for b in blobs:
        offs.append(pos); pos += len(b)
    song = [len(ORDER)]
    for o in offs: song += [o & 0xFF, o >> 8]
    song += [idx[e] for e in CH_ENTRY]
    for b in blobs: song += b
    return song, idx, offs, blobs

I_BARS, B_BARS = 8, 96
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
        print('FAIL: stop/restart'); ok = False
    if maxstack > 7: print('FAIL: stack'); ok = False
    nz = lambda r: [(v, f if v else 0) for v, f in r]
    for t in range(I + 2 * B, I + 3 * B):
        if nz(TL[t]) != nz(TL[t - B]):
            print(f'FAIL: tick {t} differs from {t - B}'); ok = False; break
    def body(k):
        a = I + B * k
        return [(e[0] - a,) + e[1:] for e in ev if a <= e[0] < a + B]
    print('body passes equal:', body(0) == body(1) == body(2), len(body(0)))
    ok &= body(1) == body(2)
    for n in range(3):
        ns = [e[3] for e in ev if e[1] == n and e[2] == 'note' and e[3] > 0]
        print(f'ch{n} range {note_name(min(ns))}..{note_name(max(ns))}')
        if min(ns) < 1 or max(ns) > 63: ok = False
    print('vol max', max(t[n][0] for t in TL for n in range(4)))
    # per-channel body length check via delay sums is implied by pass equality; print time
    print(f'first pass {(I + B) / TEMPO:.1f}s  body {B / TEMPO:.1f}s')
    print('OK' if ok else 'PROBLEMS')
    if ok:
        open('./orbital.bin', 'wb').write(bytes(song))
