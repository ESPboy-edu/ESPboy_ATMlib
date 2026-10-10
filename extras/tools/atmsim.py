#!/usr/bin/env python3
"""Simulator of ATMlib's playroutine (control flow + per-tick channel state). From LearnATMlib.md Appendix A."""
NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
TIMELINE = []
DELAYLOG = []
STOPLOG = []

def note_name(n):
    if n <= 0:
        return '--'
    n0 = n - 1
    return f"{NOTE_NAMES[n0 % 12]}{2 + n0 // 12}"

def freq_hz(note):
    if note <= 0 or note > 63:
        return 0
    return int(round(440 * 2 ** ((note - 10) / 12)))

def s8(v):
    return v - 256 if v > 127 else v

class Ch:
    def __init__(self):
        self.ptr = 0; self.note = 0; self.stack = []; self.repeatPoint = 0
        self.delay = 0; self.counter = 0; self.track = 0; self.vol = 0; self.freq = 0
        self.reCount = 0; self.reConfig = 0; self.slide = 0; self.slideCfg = 0; self.slideCount = 0
        self.arpNotes = 0; self.arpTiming = 0; self.arpCount = 0; self.arp = None; self.tra = 0
        self.treD = 0; self.treC = 0; self.treN = 0; self.glis = 0; self.glisCount = 0
        self.stopped = False

def parse(song):
    ntr = song[0]
    offs = [song[1 + 2 * i] | (song[2 + 2 * i] << 8) for i in range(ntr)]
    pos = 1 + 2 * ntr
    entries = list(song[pos:pos + 4])
    base = pos + 4
    return ntr, offs, entries, base

def simulate(song, max_ticks, trace=True):
    global TIMELINE
    TIMELINE = []
    DELAYLOG.clear(); STOPLOG.clear()
    ntr, offs, entries, base = parse(song)
    ch = [Ch() for _ in range(4)]
    for n in range(4):
        ch[n].ptr = base + offs[entries[n]]
    events = []; restarts, stops = [], []
    maxstack = 0; tempo = 25; tempo_log = []; tick = 0
    while tick < max_ticks:
        for n in range(4):
            c = ch[n]
            if c.reConfig:
                if c.reCount >= (c.reConfig & 3): c.reCount = 0
                else: c.reCount += 1
            if c.glis:
                cfg = c.glis & 0xFF
                if c.glisCount >= (cfg & 0x7F):
                    c.note += -1 if cfg & 0x80 else 1
                    c.note = max(1, min(63, c.note)); c.freq = freq_hz(c.note); c.glisCount = 0
                else: c.glisCount += 1
            if c.slide:
                if not c.slideCount:
                    isf = bool(c.slideCfg & 0x40)
                    vf = (c.freq if isf else c.vol) + c.slide
                    if not (c.slideCfg & 0x80):
                        if vf < 0: vf = 0
                        elif isf and vf > 9397: vf = 9397
                    if isf: c.freq = vf
                    else: c.vol = vf & 0xFF
                if c.slideCount >= (c.slideCfg & 0x3F): c.slideCount = 0
                else: c.slideCount += 1
            if c.arpNotes and c.note:
                if (c.arpCount & 0x1F) < (c.arpTiming & 0x1F):
                    c.arpCount += 1
                else:
                    if (c.arpCount & 0xE0) == 0x00: c.arpCount = 0x20
                    elif (c.arpCount & 0xE0) == 0x20 and not (c.arpTiming & 0x40) and c.arpNotes != 0xFF: c.arpCount = 0x40
                    else: c.arpCount = 0x00
                    an = c.note
                    if (c.arpCount & 0xE0) != 0x00:
                        an = 0 if c.arpNotes == 0xFF else an + (c.arpNotes >> 4)
                    if (c.arpCount & 0xE0) == 0x40: an += (c.arpNotes & 0x0F)
                    if c.arpNotes == 0xFF and an == 0 and c.tra != 0:
                        raise Exception('ATM_CUT with transposition != 0')
                    c.freq = freq_hz(an + c.tra)
            if c.treD:
                isf = bool(c.treC & 0x40)
                vt = c.freq if isf else c.vol
                vt = vt + c.treD if (c.treN & 0x80) else vt - c.treD
                if vt < 0: vt = 0
                elif isf and vt > 9397: vt = 9397
                if isf: c.freq = vt
                else: c.vol = vt & 0xFF
                if (c.treN & 0x1F) < (c.treC & 0x1F): c.treN += 1
                else: c.treN = 0 if (c.treN & 0x80) else 0x80
            if c.delay:
                if c.delay != 0xFFFF: c.delay -= 1
            else:
                while True:
                    cmd = song[c.ptr]; c.ptr += 1
                    if cmd < 64:
                        c.note = cmd
                        if cmd: c.note = (cmd + c.tra) & 0xFF
                        c.freq = freq_hz(c.note)
                        if c.slideCfg == 0: c.vol = c.reCount
                        if c.arpTiming & 0x20: c.arpCount = 0
                        events.append((tick, n, 'note', c.note, c.vol, c.arp, c.tra))
                    elif cmd < 160:
                        fx = cmd - 64
                        if fx == 0:
                            c.vol = song[c.ptr]; c.ptr += 1; c.reCount = c.vol
                        elif fx in (1, 4):
                            c.slide = s8(song[c.ptr]); c.ptr += 1; c.slideCfg = 0 if fx == 1 else 0x40
                        elif fx in (2, 5):
                            c.slide = s8(song[c.ptr]); c.slideCfg = song[c.ptr + 1]; c.ptr += 2
                            if fx == 5: c.slideCfg |= 0x40
                        elif fx in (3, 6): c.slide = 0
                        elif fx == 7:
                            c.arpNotes = song[c.ptr]; c.arpTiming = song[c.ptr + 1]
                            c.arp = (c.arpNotes, c.arpTiming); c.ptr += 2
                        elif fx in (8, 21): c.arpNotes = 0; c.arp = None
                        elif fx == 9: c.reConfig = song[c.ptr]; c.ptr += 1
                        elif fx == 10: c.reConfig = 0
                        elif fx == 11: c.tra = s8((c.tra + s8(song[c.ptr])) & 0xFF); c.ptr += 1
                        elif fx == 12: c.tra = s8(song[c.ptr]); c.ptr += 1
                        elif fx == 13: c.tra = 0
                        elif fx in (14, 16):
                            c.treD = song[c.ptr]; c.treC = song[c.ptr + 1] + (0 if fx == 14 else 0x40); c.ptr += 2
                        elif fx in (15, 17): c.treD = 0
                        elif fx == 18: c.glis = song[c.ptr]; c.ptr += 1
                        elif fx == 19: c.glis = 0
                        elif fx == 20:
                            c.arpNotes = 0xFF; c.arpTiming = song[c.ptr]; c.ptr += 1; c.arp = ('cut', c.arpTiming)
                        elif fx == 22: raise Exception("effect 22 (0x56) does not exist in ATMlib")
                        elif fx == 23:
                            events.append((tick, n, 'cue', song[c.ptr])); c.ptr += 1
                        elif cmd == 156:
                            tempo = (tempo + song[c.ptr]) & 0xFF; c.ptr += 1; tempo_log.append((tick, tempo))
                        elif cmd == 157:
                            tempo = song[c.ptr]; c.ptr += 1; tempo_log.append((tick, tempo))
                        elif cmd == 158:
                            for i in range(4):
                                ch[i].repeatPoint = song[c.ptr]; c.ptr += 1
                        elif cmd == 159:
                            c.vol = 0; c.delay = 0xFFFF; c.stopped = True; stops.append((tick, n)); STOPLOG.append((tick, n, c.ptr - 1))
                        else: raise Exception(f"unmodelled FX {cmd:#x} at {c.ptr-1}")
                    elif cmd < 224:
                        c.delay = cmd - 159; DELAYLOG.append((tick, n, c.ptr - 1))
                    elif cmd == 224:                      # LONG DELAY: variable-length number (7 bits per byte) + 65
                        q = 0
                        while True:
                            d = song[c.ptr]; c.ptr += 1
                            q = (q << 7) | (d & 0x7F)
                            if not d & 0x80: break
                        c.delay = q + 65; DELAYLOG.append((tick, n, c.ptr - 2))
                    elif cmd in (252, 253):
                        newc = 0 if cmd == 252 else song[c.ptr]
                        if cmd == 253: c.ptr += 1
                        newt = song[c.ptr]; c.ptr += 1
                        if newt != c.track:
                            c.stack.append((c.counter, c.track, c.ptr - base))
                            if len(c.stack) > 7: raise Exception("call stack overflow (>7)")
                            maxstack = max(maxstack, len(c.stack))
                            c.track = newt
                        c.counter = newc
                        c.ptr = base + offs[c.track]
                    elif cmd == 254:
                        if c.counter > 0 or len(c.stack) == 0:
                            if c.counter: c.counter -= 1
                            c.ptr = base + offs[c.track]
                        else:
                            cnt, trk, p = c.stack.pop()
                            c.ptr = p + base; c.counter = cnt; c.track = trk
                    else: raise Exception(f"bad cmd {cmd} at {c.ptr-1}")
                    if c.delay != 0: break
                if c.delay != 0xFFFF: c.delay -= 1
            if all(x.stopped for x in ch):
                if sum(x.repeatPoint for x in ch):
                    restarts.append((tick, n))
                    for k in range(4):
                        ch[k].ptr = base + offs[ch[k].repeatPoint]
                        ch[k].delay = 0; ch[k].stopped = False
                else:
                    TIMELINE.append([(x.vol, x.freq) for x in ch])
                    simulate.tempo_log = tempo_log
                    return events, restarts, stops, maxstack, tempo
        TIMELINE.append([(x.vol, x.freq) for x in ch])
        tick += 1
    simulate.tempo_log = tempo_log
    return events, restarts, stops, maxstack, tempo
