# Generates pokeSfx.h and atmSfx.h and the effect table (same 45 effects in both lists).
# (display name, var, channel, poke segments (m,Hz,slide,frames), ATM tokens)
N="ATM_NOTE_"
FX=[
("Jump","Jump",1,[(2,300,60,10)],"ATM_VOL(50), ATM_SL_VOL((uint8_t)-3), ATM_GLIS(0), ATM_NOTE_C4, ATM_DELAY(14)"),
("Coin","Coin",1,[(3,1320,0,4),(2,1760,0,14)],"ATM_VOL(45), ATM_SL_VOL((uint8_t)-3), ATM_NOTE_B5, ATM_DELAY(2), ATM_NOTE_E6, ATM_DELAY(12)"),
("Laser","Laser",0,[(2,3000,-186,14)],"ATM_VOL(55), ATM_SL_FRQ((uint8_t)-100), ATM_NOTE_C7, ATM_DELAY(14)"),
("Hit","Hit",3,[(3,1000,0,8)],"ATM_VOL(55), ATM_SL_VOL((uint8_t)-7), ATM_NOTE_C5, ATM_DELAY(8)"),
("Explode","Explode",3,[(1,500,0,40)],"ATM_VOL(63), ATM_SL_VOL((uint8_t)-2), ATM_NOTE_C3, ATM_DELAY(32)"),
("PowerUp","PowerUp",1,[(3,523,0,4),(3,659,0,4),(2,784,0,14)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-2), ATM_NOTE_C5, ATM_DELAY(2), ATM_NOTE_E5, ATM_DELAY(2), ATM_NOTE_G5, ATM_DELAY(2), ATM_NOTE_C6, ATM_DELAY(12)"),
("Select","Select",0,[(3,880,0,4)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-5), ATM_NOTE_A5, ATM_DELAY(6)"),
("Fail","Fail",1,[(2,440,-11,30)],"ATM_VOL(45), ATM_SL_VOL((uint8_t)-1), ATM_GLIS(0x81), ATM_NOTE_A3, ATM_DELAY(24)"),
("Blip","Blip",0,[(3,1568,0,3)],"ATM_VOL(35), ATM_SL_VOL((uint8_t)-6), ATM_NOTE_E6, ATM_DELAY(5)"),
("Whoosh","Whoosh",0,[(2,200,100,12)],"ATM_VOL(2), ATM_SL_VOL(5), ATM_NOTE_C4, ATM_DELAY(7), ATM_SL_VOL((uint8_t)-6), ATM_DELAY(8)"),
("Siren","Siren",0,[(3,600,25,12),(3,900,-25,12)],"ATM_VOL(35), ATM_NOTE_A4, ATM_DELAY(4), ATM_NOTE_E5, ATM_DELAY(4), ATM_NOTE_A4, ATM_DELAY(4), ATM_NOTE_E5, ATM_DELAY(4), ATM_NOTE_A4, ATM_DELAY(4), ATM_NOTE_E5, ATM_DELAY(4)"),
("Gem","Gem",0,[(3,2093,0,3),(3,2637,0,3),(3,3136,0,3),(2,4186,0,14)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_ARP(0x43, 0x20), ATM_NOTE_C6, ATM_DELAY(16)"),
("Bounce","Bounce",2,[(3,180,0,6),(2,180,0,6),(1,180,0,6)],"ATM_SL_VOL((uint8_t)-3), ATM_VOL(60), ATM_NOTE_G2, ATM_DELAY(5), ATM_VOL(40), ATM_NOTE_G2, ATM_DELAY(4), ATM_VOL(24), ATM_NOTE_G2, ATM_DELAY(3), ATM_VOL(12), ATM_NOTE_G2, ATM_DELAY(3)"),
("Warp","Warp",1,[(2,100,30,28)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_GLIS(0), ATM_NOTE_C3, ATM_DELAY(30)"),
("PowerDown","PowerDn",1,[(3,784,0,4),(3,659,0,4),(3,523,0,4),(2,392,0,14)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-2), ATM_NOTE_C6, ATM_DELAY(3), ATM_NOTE_G5, ATM_DELAY(3), ATM_NOTE_E5, ATM_DELAY(3), ATM_NOTE_C5, ATM_DELAY(10)"),
("Alarm","Alarm",0,[(3,880,0,6),(3,660,0,6),(3,880,0,6),(3,660,0,6)],"ATM_VOL(40), ATM_TREM(12, 1), ATM_NOTE_A5, ATM_DELAY(7), ATM_NOTE_F5, ATM_DELAY(7)"),
("Click","Click",0,[(3,2000,0,3)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-20), ATM_NOTE_C6, ATM_DELAY(2)"),
("Thud","Thud",2,[(3,120,-4,8)],"ATM_VOL(63), ATM_SL_VOL((uint8_t)-6), ATM_GLIS(0x80), ATM_NOTE_G2, ATM_DELAY(9)"),
("Zip","Zip",0,[(2,4000,-200,12)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-4), ATM_GLIS(0x80), ATM_NOTE_C7, ATM_DELAY(10)"),
("Magic","Magic",0,[(3,1568,10,4),(3,2093,10,4),(3,2637,10,4),(2,3136,10,14)],"ATM_VOL(36), ATM_SL_VOL((uint8_t)-1), ATM_VIB(4, 1), ATM_NOTE_G6, ATM_DELAY(8), ATM_NOTE_C7, ATM_DELAY(14)"),
# ---- NES style: pulse arpeggios, fast pitch drops, noise puffs
("NES Pick","nPick",0,[(3,659,0,2),(3,784,0,2),(3,1319,0,2),(2,1568,0,10)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-2), ATM_NOTE_E5, ATM_DELAY(1), ATM_NOTE_G5, ATM_DELAY(1), ATM_NOTE_E6, ATM_DELAY(1), ATM_NOTE_G6, ATM_DELAY(9)"),
("NES Shot","nShot",0,[(3,1800,-150,8)],"ATM_VOL(50), ATM_SL_FRQ((uint8_t)-110), ATM_NOTE_G6, ATM_DELAY(8)"),
("NES Stomp","nStomp",1,[(3,300,-20,6),(2,150,-5,10)],"ATM_VOL(55), ATM_SL_VOL((uint8_t)-5), ATM_GLIS(0x80), ATM_NOTE_G3, ATM_DELAY(10)"),
("NES 1UP","n1up",1,[(3,659,0,4),(3,784,0,4),(3,1319,0,4),(3,1047,0,4),(3,1175,0,4),(2,1568,0,14)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_E5, ATM_DELAY(2), ATM_NOTE_G5, ATM_DELAY(2), ATM_NOTE_E6, ATM_DELAY(2), ATM_NOTE_C6, ATM_DELAY(2), ATM_NOTE_D6, ATM_DELAY(2), ATM_NOTE_G6, ATM_DELAY(12)"),
("NES Pipe","nPipe",2,[(2,800,-18,30)],"ATM_VOL(45), ATM_SL_VOL((uint8_t)-1), ATM_GLIS(0x80), ATM_NOTE_C5, ATM_DELAY(20)"),
("NES Fire","nFire",3,[(2,500,0,18)],"ATM_VOL(45), ATM_SL_VOL((uint8_t)-3), ATM_NOTE_C4, ATM_DELAY(14)"),
("NES Block","nBlock",0,[(3,220,0,3),(3,165,0,3)],"ATM_VOL(50), ATM_SL_VOL((uint8_t)-8), ATM_NOTE_A3, ATM_DELAY(2), ATM_NOTE_E3, ATM_DELAY(6)"),
("NES Dash","nDash",3,[(3,500,0,5),(2,500,0,8)],"ATM_VOL(2), ATM_SL_VOL(8), ATM_NOTE_E4, ATM_DELAY(3), ATM_SL_VOL((uint8_t)-8), ATM_DELAY(5)"),
("NES Dead","nDead",1,[(3,988,0,4),(2,660,-12,24)],"ATM_VOL(45), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_B5, ATM_DELAY(4), ATM_GLIS(0x80), ATM_NOTE_B4, ATM_DELAY(20)"),
("NES Beep","nBeep",1,[(3,1000,0,3),(3,1000,0,3)],"ATM_VOL(35), ATM_SL_VOL((uint8_t)-2), ATM_NOTE_B5, ATM_DELAY(3), ATM_NOTE_B5, ATM_DELAY(3)"),
# ---- Atari 2600 style: buzzy, low, coarse
("ATA Pew","tPew",0,[(3,2400,-120,10)],"ATM_VOL(55), ATM_SL_FRQ((uint8_t)-110), ATM_NOTE_G6, ATM_DELAY(10)"),
("ATA Crash","tCrash",3,[(1,500,0,48)],"ATM_VOL(63), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_C2, ATM_DELAY(48)"),
("ATA Buzz","tBuzz",2,[(2,110,0,24)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_TREM(10, 0), ATM_NOTE_A2, ATM_DELAY(14)"),
("ATA Bomb","tBomb",2,[(2,200,-5,30)],"ATM_VOL(60), ATM_SL_VOL((uint8_t)-2), ATM_GLIS(0x81), ATM_NOTE_C3, ATM_DELAY(24)"),
("ATA Blip","tBlip",0,[(3,440,0,4)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-6), ATM_NOTE_A4, ATM_DELAY(5)"),
("ATA Zap","tZap",0,[(3,900,100,8)],"ATM_VOL(45), ATM_SL_FRQ(60), ATM_NOTE_A4, ATM_DELAY(8)"),
("ATA Lose","tLose",2,[(3,262,0,6),(3,220,0,6),(3,185,0,6),(2,147,0,16)],"ATM_VOL(45), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_C4, ATM_DELAY(4), ATM_NOTE_A3, ATM_DELAY(4), ATM_NOTE_F3_, ATM_DELAY(4), ATM_NOTE_D3, ATM_DELAY(10)"),
("ATA Rumb","tRumb",2,[(2,55,0,30)],"ATM_VOL(55), ATM_SL_VOL((uint8_t)-2), ATM_VIB(6, 0), ATM_NOTE_C2, ATM_DELAY(28)"),
("ATA Tick","tTick",0,[(3,1500,0,2)],"ATM_VOL(35), ATM_SL_VOL((uint8_t)-18), ATM_NOTE_C6, ATM_DELAY(2)"),
("ATA Win","tWin",0,[(3,262,0,5),(3,330,0,5),(3,392,0,5),(2,523,0,14)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_C4, ATM_DELAY(3), ATM_NOTE_E4, ATM_DELAY(3), ATM_NOTE_G4, ATM_DELAY(3), ATM_NOTE_C5, ATM_DELAY(10)"),
# ---- Sega style: bright, wide, bell-like
("SEG Ring","gRing",0,[(3,1760,0,4),(2,2637,0,18)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_A5, ATM_DELAY(3), ATM_NOTE_E6, ATM_DELAY(14)"),
("SEG Spin","gSpin",1,[(2,200,40,20)],"ATM_VOL(35), ATM_SL_FRQ(30), ATM_NOTE_C3, ATM_DELAY(24)"),
("SEG Bump","gBump",1,[(3,500,-30,5),(3,330,0,4)],"ATM_VOL(50), ATM_SL_VOL((uint8_t)-5), ATM_GLIS(0x80), ATM_NOTE_C5, ATM_DELAY(8)"),
("SEG Boing","gBoing",0,[(2,300,60,12),(2,700,-40,12)],"ATM_VOL(40), ATM_SL_VOL((uint8_t)-1), ATM_VIB(10, 1), ATM_NOTE_C4, ATM_DELAY(20)"),
("SEG Chime","gChime",0,[(3,1047,0,4),(3,1319,0,4),(3,1568,0,4),(2,2093,0,20)],"ATM_VOL(36), ATM_SL_VOL((uint8_t)-1), ATM_NOTE_C6, ATM_DELAY(4), ATM_NOTE_E6, ATM_DELAY(4), ATM_NOTE_G6, ATM_DELAY(4), ATM_NOTE_C7, ATM_DELAY(14)"),
]
assert len({f[1] for f in FX})==len(FX) and all(len(f[0])<=9 for f in FX)
for n,v,ch,seg,_ in FX:
    for m,hz,sl,ln in seg:
        end=hz+sl*ln
        if ch!=3: assert 40<=hz<=7800 and 40<=end<=7800,(n,hz,end)
        assert abs(sl)<=242 and 0<=m<=3 and 1<=ln<=255,n
def cap(v): return v[0].upper()+v[1:]
o=['// %d "poke" effects. Each is a list of SFX_SEG(m, Hz, slide Hz/frame, frames) + SFX_END.\n// sizeof() of each array is the byte size shown on screen (4 per segment + 1).\n// Generated by extras/tools/gen_sfx_demo.py - edit that and re-run, or edit here by hand.\n#ifndef POKESFX_H\n#define POKESFX_H\n#include "sfx.h"\n\n#define H SFX_HZ\n#define S SFX_SL\n'%len(FX)]
for n,v,ch,seg,_ in FX:
    body=", ".join("SFX_SEG(%d, H(%d), S(%d), %d)"%s for s in seg)
    o.append("static const uint8_t p%s[] PROGMEM = { %s, SFX_END };   // %s\n"%(cap(v),body,n))
o.append("\n#undef H\n#undef S\n#endif\n")
open("pokeSfx.h","w").write("".join(o))
o=['// %d effects for ATM.playSfx(track, channel). Each is a normal one-track ATMlib command stream;\n// ATM_SFX_TRACK() appends ATM_STOP_CHAN. sizeof() = the byte size shown on screen.\n// Timing is in song ticks, so the effects get faster or slower with the song tempo (36..44 here).\n// One track only: no GOTO/REPEAT. One slide slot per channel (SL_VOL or SL_FRQ, not both).\n#ifndef ATMSFX_H\n#define ATMSFX_H\n\n'%len(FX)]
for n,v,ch,seg,t in FX:
    o.append("ATM_SFX_TRACK(a%s, %s);   // %s\n"%(cap(v),t,n))
o.append("\n#endif\n")
open("atmSfx.h","w").write("".join(o))
tab=["#define COUNT %d\n\n"%len(FX)]
for lst,pre in (("pokeList","p"),("atmList","a")):
    tab.append("static const SfxInfo %s[] PROGMEM = {\n"%lst)
    for n,v,ch,seg,_ in FX: tab.append('  FX("%s", %s%s, %d),\n'%(n,pre,cap(v),ch))
    tab.append("};\n\n")
open("fxtable.inc","w").write("".join(tab))
print(len(FX))
