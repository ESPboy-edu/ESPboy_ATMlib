#!/usr/bin/env python3
"""
convert.py - convert an old ATMlib loop (ATM_STOP_CHAN + ATM_GOTO_ADV, one silent tick per loop) to the
hop-free self-loop (every channel's entry track ends with ATM_GOTO(itself)).

Usage:   ATM_SRC=/path/to/ATMlib/src python3 convert.py old_song.h new_song.h
Needs:   g++, atmsim.py in the current folder.  The song.h must have one ATM_* command per line and
         '//"Track N"' or '// Track N:' marker lines (every file this project wrote does).
What it does
  1. compiles the old header against the real ATMcmds.h and checks its own parse reproduces the bytes
  2. simulates the old song to the first restart; finds, for each channel, the LAST delay it executed
     and the STOP in its entry track
  3. last delay + 1 (the old loop was L-1 ticks of delay + 1 restart tick), STOP -> ATM_GOTO(entry),
     deletes ATM_GOTO_ADV
  4. merges tracks that became identical (the old '_last' variants), renumbers, rewrites the address table
  5. verifies: no stop, no restart, same notes as the old first loop, timeline repeats with period L
"""
import re, subprocess, sys, os, tempfile
sys.path.insert(0, '.')
import atmsim

# Folder with ATMcmds.h (the real library header).  Run from the folder that holds atmsim.py.
SRC = os.environ.get('ATM_SRC', '../src')

def run_dump(cpp, tag):
    with tempfile.TemporaryDirectory() as d:
        src = f'{d}/a.cpp'
        open(src, 'w').write(cpp)
        r = subprocess.run(['g++', '-include', 'stdint.h', '-DPROGMEM=', f'-I{SRC}', src, '-o', f'{d}/a'],
                           capture_output=True, text=True)
        if r.returncode:
            raise Exception(r.stderr[:2000])
        return subprocess.run([f'{d}/a'], capture_output=True, text=True).stdout


def compile_song(path, name):
    cpp = f'#include "{os.path.abspath(path)}"\n#include <stdio.h>\nint main(){{for(unsigned i=0;i<sizeof({name});i++)printf("%d ",{name}[i]);}}'
    return [int(x) for x in run_dump(cpp, name).split()]

def compile_lines(lines):
    """each line = one ATM_ command text (no comment, trailing comma ok). returns list of byte lists."""
    cpp = '#include <ATMcmds.h>\n#include <stdio.h>\n#define Song const uint8_t\n'
    for i, l in enumerate(lines):
        cpp += f'static const uint8_t L{i}[] = {{ {l.rstrip().rstrip(",")} }};\n'
    cpp += 'int main(){\n'
    for i in range(len(lines)):
        cpp += f'printf("%d:", (int)sizeof(L{i})); for(unsigned k=0;k<sizeof(L{i});k++)printf(" %d", L{i}[k]); printf("\\n");\n'
    cpp += '}\n'
    out = run_dump(cpp, 'lines').strip().split('\n')
    res = []
    for o in out:
        a, b = o.split(':')
        res.append([int(x) for x in b.split()])
    return res

def code_of(line):
    return re.sub(r'//.*', '', line).strip()

MARK = re.compile(r'^\s*//\s*"?Track\s+(\d+)')

class Song:
    pass

def parse_file(path):
    L = open(path).read().split('\n')
    si = next(i for i, l in enumerate(L) if re.match(r'\s*Song\s+\w+\[\]', l))
    name = re.match(r'\s*Song\s+(\w+)\[\]', L[si]).group(1)
    ntr = int(re.match(r'\s*0x([0-9A-Fa-f]{2})', L[si + 1]).group(1), 16)
    s = Song(); s.name = name; s.pre = L[:si]; s.ntrline = L[si + 1]; s.songline = L[si]; s.ntr = ntr
    s.addr = L[si + 2: si + 2 + ntr]
    i = si + 2 + ntr
    while not re.match(r'\s*0x[0-9A-F]{2},\s*//', L[i]): i += 1
    s.mid = L[si + 2 + ntr:i]          # blank lines between table and entries
    s.entl = L[i:i + 4]
    i += 4
    end = next(k for k in range(i, len(L)) if L[k].startswith('};'))
    s.tail = L[end:]
    body = L[i:end]
    # split to blocks
    s.blocks = []   # dict: head(lines before cmds incl marker), cmds[(text)], trail(lines)
    cur = None
    for l in body:
        m = MARK.match(l)
        if m:
            cur = dict(idx=int(m.group(1)), head=[l], cmds=[], trail=[])
            s.blocks.append(cur)
        elif cur is None:
            s.pre_body = getattr(s, 'pre_body', []) + [l]
        elif code_of(l).startswith('ATM_'):
            assert not cur['trail'] or True
            if cur['trail']:       # command after trailing -> treat trailing as part of block? keep simple
                raise Exception('cmd after trail: ' + l)
            cur['cmds'].append(l)
        else:
            if cur['cmds']: cur['trail'].append(l)
            else: cur['head'].append(l)
    assert [b['idx'] for b in s.blocks] == list(range(ntr)), 'track marker order'
    return s

def blocks_bytes(s):
    allc = [code_of(c) for b in s.blocks for c in b['cmds']]
    bs = compile_lines(allc)
    k = 0
    for b in s.blocks:
        b['bytes'] = bs[k:k + len(b['cmds'])]; k += len(b['cmds'])

def name_of(b):
    m = re.search(r'Track\s+\d+"?:?\s+(\w+)', b['head'][0])
    return m.group(1)

def convert(path, outpath):
    s = parse_file(path)
    orig = compile_song(path, s.name)
    blocks_bytes(s)
    flat = [x for b in s.blocks for c in b['bytes'] for x in c]
    ntr, offs, entries, base = atmsim.parse(orig)
    assert orig[base:] == flat, 'body bytes mismatch'
    # offset -> (block, cmd idx)
    where = {}
    for b in s.blocks:
        p = base + offs[b['idx']]
        for ci, c in enumerate(b['bytes']):
            where[p] = (b['idx'], ci); p += len(c)
    # simulate original
    ev0, restarts, stops, mx, tempo = atmsim.simulate(orig, 3000)
    T = restarts[0][0]
    P = T + 1
    tl0 = [list(t) for t in atmsim.TIMELINE]
    dl = [d for d in atmsim.DELAYLOG if d[0] < T + 1]
    stp = [x for x in atmsim.STOPLOG if x[0] <= T]
    orig_period_events = [e for e in ev0 if e[0] < P]
    print(f'{s.name}: bytes={len(orig)} first restart at tick {T} tempo={tempo}')
    edits = {}
    for n in range(4):
        last = [d for d in dl if d[1] == n]
        lastptr = last[-1][2] if last else None
        sp = [x for x in stp if x[1] == n]
        if not sp:
            print('  ch', n, 'never stops'); continue
        sptr = sp[0][2]
        cnt = sum(1 for d in dl if d[2] == lastptr) if lastptr is not None else 0
        sb, sc = where[sptr]
        e = entries[n]
        print(f'  ch{n}: entry {e} stop in track {sb} (cmd {sc}), last delay in track {where[lastptr][0] if lastptr else None}, exec count {cnt}')
        assert sb == e, 'stop not in entry track'
        edits[n] = (lastptr, (sb, sc), e, cnt)
    # apply edits
    for n, (lp, (sb, sc), e, cnt) in edits.items():
        assert cnt == 1, f'last delay of ch{n} executed {cnt}x'
        tb, tc = where[lp]
        t = s.blocks[tb]['cmds'][tc]
        m = re.search(r'ATM_DELAY\((\d+)\)', t)
        d = int(m.group(1)); assert d < 64
        s.blocks[tb]['cmds'][tc] = t.replace(m.group(0), f'ATM_DELAY({d + 1})') + ''
        if '//' not in t: s.blocks[tb]['cmds'][tc] = s.blocks[tb]['cmds'][tc]
        # stop -> goto entry
        old = s.blocks[sb]['cmds'][sc]
        assert code_of(old).startswith('ATM_STOP_CHAN')
        ind = re.match(r'\s*', old).group(0)
        s.blocks[sb]['cmds'][sc] = f'{ind}ATM_GOTO({e}),   // loop: restart this track (no STOP, no gap)'
    # remove ADV
    for b in s.blocks:
        b['cmds'] = [c for c in b['cmds'] if not code_of(c).startswith('ATM_GOTO_ADV')]
    # merge duplicates
    names = {b['idx']: name_of(b) for b in s.blocks}
    entries = list(entries)
    def refs_re():
        return re.compile(r'ATM_(GOTO|REPEAT)\((?:(\d+)(\s*,\s*))?(\d+)\)')
    def sig(b, alive):
        out = []
        for c in b['cmds']:
            out.append(re.sub(r'\s*//.*', '', c).strip())
        return tuple(out)
    merged = []
    while True:
        sigs = {}
        found = None
        for b in s.blocks:
            if b['idx'] == 0: continue
            sg = sig(b, None)
            if sg in sigs:
                found = (sigs[sg], b['idx']); break
            sigs[sg] = b['idx']
        if not found: break
        keep, rem = found
        merged.append((names[rem], names[keep]))
        s.blocks = [b for b in s.blocks if b['idx'] != rem]
        def fix(i):
            i = keep if i == rem else i
            return i - 1 if i > rem else i
        rx = refs_re()
        for b in s.blocks:
            nc = []
            for c in b['cmds']:
                def sub(m):
                    a, tr = m.group(2), int(m.group(4))
                    nt = fix(tr)
                    return f'ATM_{m.group(1)}(' + (f'{a}{m.group(3)}' if a is not None else '') + f'{nt})'
                c2 = rx.sub(sub, c)
                if c2 != c:
                    c2 = re.sub(r'-> ' + re.escape(names[rem]) + r'\b', '-> ' + names[keep], c2)
                    c2 = re.sub(r'(\d+x )' + re.escape(names[rem]) + r'\b', r'\g<1>' + names[keep], c2)
                nc.append(c2)
            b['cmds'] = nc
        entries = [fix(x) for x in entries]
        # renumber
        oldidx = [b['idx'] for b in s.blocks]
        for b in s.blocks:
            b['newidx'] = fix(b['idx'])
        for b in s.blocks:
            b['idx'] = b['newidx']
            def rn(m, b=b):
                tot = len(m.group(2)) + len(m.group(3)); d = str(b['idx'])
                return m.group(1) + ' ' * max(1, tot - len(d)) + d
            b['head'][0] = re.sub(r'(Track)(\s+)(\d+)', rn, b['head'][0], count=1)
        names = {b['idx']: name_of(b) for b in s.blocks}
        # re-align escaper style "Track  1:" widths
    # recompute bytes
    blocks_bytes(s)
    for b in s.blocks:
        sz = sum(len(c) for c in b['bytes'])
        b['head'][0] = re.sub(r'\[\d+b\]', f'[{sz}b]', b['head'][0])
    ntr2 = len(s.blocks)
    offs2 = []; p = 0
    for b in s.blocks:
        offs2.append(p); p += sum(len(c) for c in b['bytes'])
    total = 1 + 2 * ntr2 + 4 + p
    # build text
    out = list(s.pre)
    # preamble sizes unchanged here; handled later
    sl = re.sub(r'total song bytes = \d+', f'total song bytes = {total}', s.songline)
    out.append(sl)
    out.append(re.sub(r'0x[0-9A-Fa-f]{2}', f'0x{ntr2:02X}', s.ntrline, count=1))
    fmtB = 'Track' in s.addr[0] and '@' in s.addr[0]
    for i, b in enumerate(s.blocks):
        nm = names[i]
        o = offs2[i]
        if fmtB:
            cm = f'// Track {i:<2} @ {o:>4}  ({nm})'
        else:
            cm = f'// Address of track {i:<2} {o:>5}   {nm}'
        out.append(f'  0x{o & 255:02X}, 0x{o >> 8:02X},                 {cm}')
    out.extend(s.mid)
    for n in range(4):
        out.append(re.sub(r'0x[0-9A-F]{2}', f'0x{entries[n]:02X}', s.entl[n], count=1))
    # entry comment track numbers
    for k in range(4):
        out[-4 + k] = re.sub(r'(track )(\d+)', lambda m: m.group(1) + str(entries[k]), out[-4 + k])
    out.extend(getattr(s, 'pre_body', []))
    for b in s.blocks:
        out.extend(b['head']); out.extend(b['cmds']); out.extend(b['trail'])
    out.extend(s.tail)
    open(outpath, 'w').write('\n'.join(out))
    print('  merged:', merged)
    # verify
    new = compile_song(outpath, s.name)
    assert len(new) == total, (len(new), total)
    ev1, rs1, st1, mx1, tempo1 = atmsim.simulate(new, P * 3)
    tl1 = [list(t) for t in atmsim.TIMELINE]
    st_bad = [x for x in st1 if x[0] > 0]
    assert not rs1, 'restarts'
    assert not st_bad, st_bad
    nz = lambda r: [(v, f if v else 0) for v, f in r]
    for t in range(P, 3 * P):
        assert nz(tl1[t]) == nz(tl1[t - P]), f"timeline differs at {t}"
    for t in range(T):
        assert tl1[t] == tl0[t], f'new != orig at {t}: {tl1[t]} {tl0[t]}'
    e0 = [(a, b, c, d, e) for (a, b, c, d, e, *_) in orig_period_events if c == 'note']
    e1 = [(a, b, c, d, e) for (a, b, c, d, e, *_) in ev1 if a < P and c == 'note']
    assert e0 == e1, 'note events differ'
    print(f'  OK  {len(orig)} -> {total} bytes, period {P} ticks, max stack {mx1}, tempo {tempo1}')
    return total

if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2])
