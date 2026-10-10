import gen_orbital as g
song, idx, offs, blobs = g.assemble()
total = len(song)
H = '''#ifndef ORBITALRUSH_H
#define ORBITALRUSH_H

// ---------------------------------------------------------------------------
//  "Orbital Rush" - an original demoscene style track for ATMlib           {total} bytes
//  (driving D minor, fast arpeggios, big melodic hooks, a key-change finale)
//
//  Play with:   #include "song.h"      ATM.play(orbitalRush);
//
//  NEVER ENDS: an 8-bar intro plays once, then the 96-bar body (about 2:46) loops forever.
//  The first pass is intro + body = 104 bars = about 3:00.
//
//  Timing : ATM_SET_TEMPO({tempo}) (doubled) = about 139 BPM.  1 beat = 16 ticks, 1 bar = 64 ticks.
//  Loop   : hop-free self-loop (see LearnATMlib.md): every channel's entry track plays its intro once
//           and then calls its LOOP track, which calls the body and then calls ITSELF.
//           Every channel's body is exactly 6144 ticks (96 bars).  The body sets volume, slide,
//           arpeggio and transposition itself, so every pass is identical.
//           The riser glissando uses a tick amount that divides its length (a glissando counter
//           that does not divide it would carry over and shift the next pass by a tick).
//
//  FORM                                                            bars
//     INTRO (once)   pad fades in, drums build, lead + bass enter     8
//     A              Dm Bb F C x4   melody A1 A2 A1 A2, sparse bass  16
//     B              Gm Dm Bb C x4  melody B1 B2 B1 B2               16
//     break          Bb C Dm A x2   vibrato notes, pad, riser         8
//     chorus         Dm Bb Gm A x4  the big hook (C1 C2 C1 C2)       16
//     dance          Dm C Bb A x4   16th riff, four on the floor     16
//     lifted chorus  Em C Am B x4   same hook, +2 semitones (ADD_TRA)16
//     tag            Bb Gm C A x2   falls back into the A section     8
//
//  Channels (ATMlib defaults):
//     CH0 PULSE  - lead, absolute notes (the lift is ADD_TRA +2 around the chorus; the dance riff is
//                  transposed per chord)
//     CH1 SQUARE - arpeggio chords: pad (A, break, intro) and 3+3+2 stabs (everything else).
//                  Explicit roots (arpeggio + transposition on one channel would transpose twice).
//     CH2 SAW    - bass, ONE bar of 16ths transposed per chord (plus a sparse and a sustained pattern)
//     CH3 NOISE  - drums, four-on-the-floor in the dance, tremolo snare-roll fills
//  Note   : track 0 is only CH3's entry point and is never called from another track.
// ---------------------------------------------------------------------------

#include <ATMcmds.h>

#ifndef Song
#define Song const uint8_t PROGMEM
#endif


'''
L = [H.format(total=total, tempo=g.TEMPO), f'Song orbitalRush[] = {{     // total song bytes = {total}',
     f'  0x{len(g.ORDER):02X},                       // Number of tracks']
for i, name in enumerate(g.ORDER):
    o = offs[i]
    L.append(f'  0x{o & 0xFF:02X}, 0x{o >> 8:02X},                 // Address of track {i:<2d} {o:5d}   {name}')
L.append('')
for k, e in enumerate(g.CH_ENTRY):
    L.append(f'  0x{idx[e]:02X},                         // CH{k} entry -> track {idx[e]} ({e})')
L.append('')
for i, name in enumerate(g.ORDER):
    L.append(f'  //"Track {i}" {name}  [{len(blobs[i])}b]')
    for c in g.TR[name]:
        if c[0] == 'GOTO': L.append(f'  ATM_GOTO({idx[c[1]]}),   // -> {c[1]}')
        elif c[0] == 'REPEAT': L.append(f'  ATM_REPEAT({c[1]}, {idx[c[2]]}),   // {c[1] + 1}x {c[2]}')
        else: L.append(f'  {c[0]},')
    L.append('')
L += ['};', '', '#endif', '']
open('song.h', 'w').write('\n'.join(L))
