import re
# tiny assembler: tracks are lists of macro strings; sizes/ticks computed, header addresses generated
COST={'NOTE':1,'DELAY':1,'VOL':2,'SL_VOL':2,'SET_TRA':2,'ADD_TRA':2,'SET_TEMPO':2,'GLIS':2,'GOTO':2,'REPEAT':3,
      'ARP':3,'TREM':3,'VIB':3,'RETURN':1,'STOP_CHAN':1,'SL_FRQ':2}
def parse(t):
    m=re.match(r'ATM_(NOTE|[A-Z_]+?)(?:_[A-G]#?_?\d_?)?(?:\((.*)\))?$',t)
    name=t[4:].split('(')[0]
    if name.startswith('NOTE_'): name='NOTE'
    args=t[t.index('(')+1:-1] if '(' in t else ''
    return name,[a.strip() for a in args.split(',')] if args else []
def size(tr): return sum(COST[parse(t)[0]] for t in tr)
def ticks(tracks,i,top=None):
    top=i if top is None else top; s=0
    for t in tracks[i]:
        n,a=parse(t)
        if n=='DELAY': s+=int(a[0])
        elif n=='GOTO':
            if int(a[0])==top: return s
            s+=ticks(tracks,int(a[0]),top)
        elif n=='REPEAT': s+=(int(a[0])+1)*ticks(tracks,int(a[1]),top)
        elif n=='RETURN': return s
    return s
def emit(var,title,desc,tempo,entries,tracks,labels):
    N=len(tracks); hdr=1+2*N+4
    addr=[];a=0
    for tr in tracks: addr.append(a); a+=size(tr)
    tot=hdr+a
    L=[f"// {title}\n// {desc}\n// {tot} bytes | tempo {tempo} (ticks per second) | loop {max(ticks(tracks,e) for e in entries)} ticks\n"]
    L.append(f"Song {var}[] = {{          // total song in bytes = {tot}\n  0x{N:02X},                         // Number of tracks\n")
    for i,ad in enumerate(addr): L.append(f"  0x{ad&255:02X}, 0x{ad>>8:02X},                   // Address of track {i}   //{ad}\n")
    L.append("\n")
    for c,e in enumerate(entries): L.append(f"  0x{e:02X},                         // Channel {c} entry track\n")
    L.append("\n")
    for i,tr in enumerate(tracks):
        L.append(f'  //"Track {i}" {labels[i]}  [{size(tr)} bytes]\n')
        for t in tr: L.append("  "+re.sub(r"ATM_SL_VOL\(\(uint8_t\)-(\d+)\)",r"ATM_SL_VOL(MS(\1))",re.sub(r"ATM_VOL\((\d+)\)",r"ATM_VOL(MV(\1))",t))+",\n")
        L.append("\n")
    L.append("};\n\n")
    # verify all channel loops are equal
    loops=[ticks(tracks,e) for e in entries]
    return "".join(L),tot,loops
def notes(seq): # [(note, delay)]
    o=[]
    for n,d in seq: o+= [f"ATM_NOTE_{n}",f"ATM_DELAY({d})"]
    return o
def prog(first,callfn,offs,last_goto):
    o=[]
    for i,off in enumerate([None]+offs):
        if off is not None: o.append(f"ATM_ADD_TRA({off if off>=0 else '(uint8_t)'+str(off)})")
        o.append(callfn)
    return o
SIL=["ATM_STOP_CHAN"]
songs=[]
# ---- NES: Pipe Dream, C major, I vi IV V, pulse arpeggio hops, octave-bounce bass
def nes():
    lead=["ATM_SET_TEMPO(42)","ATM_VOL(34)","ATM_SL_VOL((uint8_t)-2)","ATM_SET_TRA(0)"]+prog(0,"ATM_GOTO(1)",[-3,-4,2],0)+["ATM_GOTO(0)"]
    motif=notes([("C5",8),("E5",8),("G5",8),("E5",8),("C6",8),("G5",8),("E5",8),("G5",8)])+["ATM_RETURN"]
    bass=["ATM_VOL(50)","ATM_SET_TRA(0)"]+prog(0,"ATM_REPEAT(3, 3)",[-3,-4,2],0)+["ATM_GOTO(2)"]
    bu=notes([("C3",8),("C4",8)])+["ATM_RETURN"]
    dr=["ATM_SL_VOL((uint8_t)-6)","ATM_REPEAT(15, 5)","ATM_GOTO(4)"]
    beat=["ATM_VOL(50)","ATM_DELAY(8)","ATM_VOL(30)","ATM_DELAY(8)","ATM_RETURN"]
    return emit("pipeDream",'"Pipe Dream" - NES style, C major (I vi IV V)',"pulse arpeggio hops over an octave-bouncing bass",42,[0,6,2,4],[lead,motif,bass,bu,dr,beat,SIL],["lead","hop motif","bass","bass bounce","drums","beat","silent"])
def atari():
    lead=["ATM_SET_TEMPO(36)","ATM_VOL(30)","ATM_VIB(3, 1)","ATM_SET_TRA(0)"]+prog(0,"ATM_GOTO(1)",[-2,-2,-1],0)+["ATM_GOTO(0)"]
    motif=notes([("A3",4),("A3",4),("C4",8),("A3",8),("E4",8),("C4",8),("D4",8),("B3",8),("E4",8)])+["ATM_RETURN"]
    bass=["ATM_VOL(52)","ATM_SET_TRA(0)"]+prog(0,"ATM_REPEAT(3, 3)",[-2,-2,-1],0)+["ATM_GOTO(2)"]
    bu=["ATM_NOTE_A2","ATM_DELAY(16)","ATM_RETURN"]
    dr=["ATM_SL_VOL((uint8_t)-9)","ATM_REPEAT(15, 5)","ATM_GOTO(4)"]
    beat=["ATM_VOL(56)","ATM_DELAY(16)","ATM_RETURN"]
    return emit("canyonRaid",'"Canyon Raid" - Atari 2600 style, A minor (Am G F E)',"wavering low square lead, saw pump bass, one noise thump per beat",36,[6,0,2,4],[lead,motif,bass,bu,dr,beat,SIL],["lead","lead motif","bass","bass pump","drums","beat","silent"])
def sega1():
    lead=["ATM_SET_TEMPO(44)","ATM_VOL(36)","ATM_SL_VOL((uint8_t)-1)","ATM_SET_TRA(0)"]+prog(0,"ATM_GOTO(1)",[-2,-5,2],0)+["ATM_GOTO(0)"]
    motif=notes([("G5",4),("E5",4),("C5",8),("E5",8),("G5",8),("C6",8),("B5",8),("G5",8),("E5",8)])+["ATM_RETURN"]
    bass=["ATM_VOL(50)","ATM_SET_TRA(0)"]+prog(0,"ATM_REPEAT(3, 3)",[-2,-5,2],0)+["ATM_GOTO(2)"]
    bu=notes([("C3",8),("G3",8)])+["ATM_RETURN"]
    dr=["ATM_SL_VOL((uint8_t)-5)","ATM_REPEAT(15, 5)","ATM_GOTO(4)"]
    beat=["ATM_VOL(52)","ATM_DELAY(8)","ATM_VOL(26)","ATM_DELAY(8)","ATM_RETURN"]
    pad=["ATM_VOL(22)","ATM_SL_VOL((uint8_t)-0)","ATM_ARP(0x47, 0x20)","ATM_NOTE_C5","ATM_DELAY(64)","ATM_NOTE_A4_","ATM_DELAY(64)","ATM_NOTE_F5","ATM_DELAY(64)","ATM_NOTE_G5","ATM_DELAY(64)","ATM_GOTO(6)"]
    return emit("greenZone",'"Green Zone" - Sega style, C major (C Bb F G)',"bright pulse lead over a syncopated bass and a hat-heavy beat",44,[0,6,2,4],[lead,motif,bass,bu,dr,beat,SIL],["lead","lead motif","bass","bass groove","drums","beat","silent"])
def sega2():
    lead=["ATM_SET_TEMPO(44)","ATM_VOL(26)","ATM_SL_VOL((uint8_t)-1)","ATM_ARP(0x34, 0x20)","ATM_NOTE_D5","ATM_DELAY(64)","ATM_ARP(0x43, 0x20)","ATM_NOTE_A4_","ATM_DELAY(64)","ATM_NOTE_F5","ATM_DELAY(64)","ATM_NOTE_C5","ATM_DELAY(64)","ATM_GOTO(0)"]
    mel=["ATM_VOL(34)","ATM_SL_VOL((uint8_t)-1)"]
    for r in ["A4","G4","F4","E4"]:
        pass
    bars=[("D5","A5"),("D5","F5"),("C5","A5"),("C5","G5")]
    for a_,c in bars: mel+= [f"ATM_NOTE_{a_}","ATM_DELAY(32)",f"ATM_NOTE_{c}","ATM_DELAY(32)"]
    mel+=["ATM_GOTO(1)"]
    bass=["ATM_VOL(50)","ATM_SET_TRA(0)"]+prog(0,"ATM_REPEAT(15, 3)",[-4,7,-5],0)+["ATM_GOTO(2)"]
    bu=["ATM_NOTE_D3","ATM_DELAY(4)","ATM_RETURN"]
    dr=["ATM_SL_VOL((uint8_t)-5)","ATM_REPEAT(15, 5)","ATM_GOTO(4)"]
    beat=["ATM_VOL(55)","ATM_DELAY(4)","ATM_VOL(20)","ATM_DELAY(4)","ATM_VOL(35)","ATM_DELAY(4)","ATM_VOL(20)","ATM_DELAY(4)","ATM_RETURN"]
    return emit("neonHighway",'"Neon Highway" - Sega style, D minor (Dm Bb F C)',"arpeggio lead, answering melody, driving 16th bass, four-on-the-floor",44,[0,1,2,4],[lead,mel,bass,bu,dr,beat],["arp lead","melody","bass","bass 16th","drums","beat"])
out="// The 4 extra songs of the sound-effect demo (the 5th, Starlit Run, is in song.h).\n#ifndef SONGS_H\n#define SONGS_H\n"+'\n// Music volume for the demo, in percent, so the sound effects stand out.\n// ATM_VOL(MV(n)) = n scaled by MUSIC_VOL_PCT (compile time, no extra bytes). 100 = original.\n#ifndef MUSIC_VOL_PCT\n#define MUSIC_VOL_PCT 60\n#endif\n#ifndef MV\n#define MV(v) ((v) * MUSIC_VOL_PCT / 100)\n#endif\n// ATM_SL_VOL(MS(n)): the decay slide scaled the same way (n = size of the fade-out step per\n// tick, at least 1), so notes keep their shape instead of dying sooner.\n#ifndef MS\n#define MS(n) ((uint8_t)-((((n) * MUSIC_VOL_PCT + 50) / 100) < 1 ? 1 : (((n) * MUSIC_VOL_PCT + 50) / 100)))\n#endif\n'+"\n"
rep=[]
for f in (nes,atari,sega1,sega2):
    t,tot,loops=f(); out+=t; rep.append((f.__name__,tot,loops))
out+="#endif\n"
open("songs.h","w").write(out)
for r in rep: print(r)
