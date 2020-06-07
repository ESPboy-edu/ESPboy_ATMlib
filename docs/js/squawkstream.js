/*
 * Emulates the Squawk Stream player
 *   setSource(data)
 *   tick()
 *   getTickCount()
 */

//var channelActiveMute = 0xF0;
//var channelActiveMute = 0b11110000;
//                        ||||||||
//                        |||||||└->  0  channel 0 is muted (0 = false / 1 = true)
//                        ||||||└-->  1  channel 1 is muted (0 = false / 1 = true)
//                        |||||└--->  2  channel 2 is muted (0 = false / 1 = true)
//                        ||||└---->  3  channel 3 is muted (0 = false / 1 = true)
//                        |||└----->  4  channel 0 is Active (0 = false / 1 = true)
//                        ||└------>  5  channel 1 is Active (0 = false / 1 = true)
//                        |└------->  6  channel 2 is Active (0 = false / 1 = true)
//                        └-------->  7  channel 3 is Active (0 = false / 1 = true)


// Note frequencies (scales with sample rate for now)
var noteTable = [
     0,
   262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494,
   523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932,  988,
  1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
  2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951,
  4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902,
  8372, 8870, 9397, 
];




function SquawkStream(sampleRate) {
  var channelActiveMute = 0xF0;
  var noRepeat = true;
  var entryBase;
  var synth;
  var data;
  var tickCount = 0;
  var tickRate  = 25;
  
  // Oscillator DMEP-type testing (Destroy My Ears, Plz)
  var testIx = 0.0;
  function DMEP() {
    testIx += 0.01;
    synth.setVolume(0, Math.floor(((Math.sin(testIx *  2) + 1.0) / 2.0) * 64));
    synth.setFrequency(0, Math.floor(((Math.sin(testIx *  3) + 1.0) / 2.0) * (8192 - 256)) + 256);
    synth.setVolume(1, Math.floor(((Math.sin(testIx *  5) + 1.0) / 2.0) * 64));
    synth.setFrequency(1, Math.floor(((Math.sin(testIx *  7) + 1.0) / 2.0) * (8192 - 256)) + 256);
    synth.setVolume(2, Math.floor(((Math.sin(testIx * 11) + 1.0) / 2.0) * 64));
    synth.setFrequency(2, Math.floor(((Math.sin(testIx * 13) + 1.0) / 2.0) * (8192 - 256)) + 256);
    synth.setVolume(3, Math.floor(((Math.sin(testIx * 17) + 1.0) / 2.0) * 64));
  }

  // Data access functions (big-endian)
  function readByte(address) { return data[address]; }
  function trackAddress(track) { return (data[(track << 1) + 1] << 8) | data[(track << 1) + 2]; }

  // Channel class - contains the core mysic data processing code
  function Channel(id) {
    var ptr         = 0;     // Pointer (into stream)
    var note        = 0;

    // Nesting
    var stackPointer = new Array();
    var stackCounter = new Array();
    var stackTrack = new Array(); // note 1
    var stackIndex   = 0;
    var repeatPoint = 0;

    // Looping
    var delay        = 0;
    var counter      = 0;     // Counter (for looping)
    var track        = 0;

    // External FX
    //var freq       = 0;
    //var vol        = 0;
    //boolean mute   = 0;

    // Volume & Frequency slide FX
    var volfreSlide  = 0;
    var volfreConfig = 0;
    var volfreCount  = 0;

    // Arpeggio FX
    var arpNotes    = 0;     // notes: base, base+[7:4], base+[7:4]+[3:0]
    var arpTiming   = 0;     // [7] = reserved, [6] = not third note ,[5] = retrigger, [4:0] = tick count
    var arpCount    = 0;

    // Retrig
    var reConfig    = 0;     // [7:2] = , [1:0] = speed
    var reCount     = 0;

    // transposition
    var transConfig  = 0;

    //Tremolo or Vibrato
    var treviDepth  = 0;
    var treviConfig = 0;
    var treviCount  = 0;

    //Glissando
    var glisConfig  = 0;
    var glisCount   = 0;

    // == BIG-ENDIAN DATA ACCESS FUNCTIONS == //
    function readByte() { return typeof data == "function" ? data() : data[ptr++]; }
    function readWord() { return (readByte() << 8) | readByte(); }
    function readVLE(address) {
      var word = 0;
      do {
        word <<= 7;
        var d = readByte();
        word |= (d & 0x7F);
      } while(d & 0x80);
      return word & 0xFFFF;
    }
    // ====================================== //

    // == INTERFACE == //
      
    // Returns channel ID, used for iteration
    function _id() {
      return id;
    };

    // Force write pointer, used during track entry point setup
    function _jumpTo(address) {
      ptr = address;
    }

    // All hail "THE PLAYROUTINE"
    function _play() {
      // Noise retriggering
      if (reConfig != 0) {
        if (reCount >= (reConfig & 0x03)) {
          synth.setFrequency(id, noteTable[reConfig >> 2]);
          reCount = 0;
        } else reCount++;
      }

      //Apply Glissando -> WORKING HURRAY :)
      if (glisConfig != 0) {
        if (glisCount >= (glisConfig & 0x7F))
        {
          if ((glisConfig & 0x80) != 0)note -= 1;
          else note += 1;
          if (note < 1) note = 1;
          else if (note > 63) note = 63;
          synth.setFrequency(id, noteTable[note]);
          glisCount = 0;
        }
        else glisCount++;
      }

      // Apply volume/frequency slides
      if (volfreSlide != 0) {
        if (volfreCount == 0) {
          if ((volfreConfig & 0x40) != 0) var vf = synth.readFrequency(id);
          else var vf = synth.readVolume(id);
          //var vf = ((volfreConfig & 0x40) != 0) ? synth.readFrequency(id) : synth.readVolume(id);
          vf += volfreSlide;
          if ((volfreConfig & 0x80) == 0) {
            if (vf < 0) vf = 0;
            else if ((volfreConfig & 0x40) !=0) {
              if (vf > 9397) vf = 9397;
            }
            else if ((volfreConfig & 0x40) == 0) {
              if (vf > 63) vf = 63;
            }
          }
          if ((volfreConfig & 0x40) != 0) vf = synth.setFrequency(id, vf);
          else vf = synth.setVolume(id, vf);
          //((volfreConfig & 0x40) !=0) ? synth.setFrequency(id, vf) : synth.setVolume(id, vf);
        }
        if (volfreCount++ >= (volfreConfig & 0x3F)) volfreCount = 0;
      }

      // Apply Arpeggio or Note Cut -> WORKING HURRAY :)
      if ((arpNotes != 0)  && (note != 0)) {
        if ((arpCount & 0x1F) < (arpTiming & 0x1F)) arpCount++;
        else {
          if ((arpCount & 0xE0) == 0) arpCount = 32;
          else if (((arpCount & 0xE0) == 32) && ((arpTiming & 0x40) == 0) && (arpNotes != 0xFF)) arpCount = 64;
          else arpCount = 0;
          var arpNote = note;
          if ((arpCount & 0xE0) != 0) {
              if (arpNotes == 0xFF) arpNote = 0;
              else arpNote += (arpNotes >> 4);
            }
          if ((arpCount & 0xE0) == 64) arpNote += (arpNotes & 15);
          synth.setFrequency(id, noteTable[arpNote + transConfig]);
        }
      }

      // Apply Tremolo or Vibrato -> WORKING HURRAY :)
      if (treviDepth != 0) {
        var vt = ((treviConfig & 0x40)!=0) ? synth.readFrequency(id) : synth.readVolume(id);
        vt = ((treviCount & 0x80)!=0) ? (vt + (treviDepth & 0x1F)) : (vt - (treviDepth & 0x1F));
        if (vt < 0) vt = 0;
        else if ((treviConfig & 0x40)!=0) if (vt > 9397) vt = 9397;
        else if ((treviConfig & 0x40)==0) if (vt > 63) vt = 63;
        ((treviConfig & 0x40)!=0) ? synth.setFrequency(id, vt) : synth.setVolume(id, vt);
        if ((treviCount & 0x1F) < (treviConfig & 0x1F)) treviCount++;
        else {
          if ((treviCount & 0x80)!=0) treviCount = 0;
          else treviCount = 0x80;
        }
      }


      if(delay != 0) { 
        if(delay !== 0xFFFF) {delay--;}
      }
      else {
        do {
          var cmd = readByte();
          if(cmd < 64) {
            // 0 … 63 : NOTE ON/OFF
            if ((note = cmd) != 0) note += transConfig;
            synth.setFrequency(id, noteTable[note]);
            if (volfreConfig == 0) synth.setVolume(id, reCount);
            if ((arpTiming & 0x20) != 0) arpCount = 0; // ARP retriggering
          } else if(cmd < 160) {
            // 64 … 159 : SETUP FX
            var fx = cmd - 64;
            switch(fx) {
              case 0: // Set volume
                synth.setVolume(id, readByte());
                reCount = synth.readVolume(id);
                break;
              case 1: // Slide volume ON
                volfreSlide = readByte();
                volfreSlide = volfreSlide > 127 ? volfreSlide - 256 : volfreSlide;
                volfreConfig = 0;
                break;
              case 2: // Slide volume ON advanced
                volfreSlide = readByte();
                volfreSlide = volfreSlide > 127 ? volfreSlide - 256 : volfreSlide;
                volfreConfig = readByte();
                break;
              case 3: // Slide volume OFF (same as 0x01 0x00)
                volfreSlide = 0;
                break;
              case 4: // Slide frequency ON
                volfreSlide = readByte();
                volfreSlide = volfreSlide > 127 ? volfreSlide - 256 : volfreSlide;
                volfreConfig = 0x40;
                break;
              case 5: // Slide frequency ON advanced
                volfreSlide = readByte();
                volfreSlide = volfreSlide > 127 ? volfreSlide - 256 : volfreSlide;
                volfreConfig = readByte() + 0x40;
                break;
              case 6: // Slide frequency OFF
                volfreSlide = 0;
                break;
              case 7: // Set Arpeggio
                arpNotes = readByte();    // 0x40 + 0x03
                arpTiming = readByte();   // 0x80 + 0x40 + 0x20 + amount
                break;
              case 8: // Arpeggio off
                arpNotes = 0;
                break;
              case 9: // Set Retriggering (noise)
                reConfig = readByte();    // RETRIG: point = 1 (*4), speed = 0 (0 = fastest, 1 = faster , 2 = fast)
                break;
              case 10: // Retriggering (noise) OFF
                reConfig = 0;
                break;
              case 11: // ADD Transposition
                transConfig += readByte();
                transConfig = transConfig > 127 ? transConfig - 256 : transConfig;
                break;
              case 12: // SET Transposition
                transConfig = readByte();
                transConfig = transConfig > 127 ? transConfig - 256 : transConfig;
                break;
              case 13: // Transposition OFF
                transConfig = 0;
                break;
              case 14: // SET Tremolo
                treviDepth = readByte();
                treviConfig = readByte();
                break;
              case 15: // Tremolo OFF
                treviDepth = 0;
                break;
              case 16: // SET Vibrato
                treviDepth = readByte();
                treviConfig = readByte() + 0x40;
                break;
              case 17: // Vibrato OFF
                treviDepth = 0;
                break;
              case 18: // Glissando
                glisConfig = readByte();
                break;
              case 19: // glissando OFF
                glisConfig = 0;
                break;
              case 20: // Note Cut
                arpNotes = 0xFF;    // 0xFF use Note Cut
                arpTiming = readByte();   // tick amount
                break;
              case 21: // Note Cut OFF
                arpNotes = 0;
                break;
              case 92: // ADD tempo
                tickRate += readByte();
                synth.setTick((sampleRate / tickRate).toFixed(0));
                break;
              case 93: // SET tempo
                tickRate = readByte();
                synth.setTick((sampleRate / tickRate).toFixed(0));
                break;
              case 94: // Goto advanced
                channel[0].repeatPoint = readByte();
                channel[1].repeatPoint = readByte();
                channel[2].repeatPoint = readByte();
                channel[3].repeatPoint = readByte();
                noRepeat = false;
                break;
              case 95: // Stop channel
                var mask = 0xF0;
                mask = mask ^ (1<<(id+4))
                channelActiveMute = channelActiveMute & mask;
                synth.setVolume(id, 0);
                delay = 0xFFFF;
                break;
              default :
                break;
            }
          } else if(cmd < 224) {
            // 160 … 223 : DELAY
            delay = cmd - 159;
          } else if(cmd == 224) {
            // 224: LONG DELAY
            delay = readVLE() + 65;
          } else if(cmd < 252) {
            // 225 … 251 : RESERVED
          } else if(cmd == 252 || cmd == 253) {
            // 252 (253) : CALL (REPEATEDLY)
            // Stack PUSH
            stackCounter[stackIndex] = counter;
            stackTrack[stackIndex] = track;
            counter = cmd == 252 ? 0 : readByte();
            track = readByte();
            stackPointer[stackIndex] = ptr;
            stackIndex++;
            ptr = trackAddress(track);
          } else if(cmd == 254) {
            // 254 : RETURN
            if(counter > 0) {
              // Repeat track
              counter--;
              ptr = trackAddress(track);
            } else {
              // Check stack depth
              if(stackIndex == 0) {
                // End-Of-File
                delay = 0xFFFF;
              } else {
                // Stack POP
                stackIndex--;
                ptr = stackPointer[stackIndex];
                counter = stackCounter[stackIndex];
                track = stackTrack[stackIndex]; // note 1
              }
            }
          } else if(cmd == 255) {
            // 255 : EMBEDDED DATA
            ptr += readVLE();
          }
        } while(delay == 0);
        // Apply volume slides
        if(delay != 0xFFFF) {delay--;}
      }
      // if all channels are inactive check for repeat
      if ((channelActiveMute & 0xF0) == 0)
      {
        var repeatSong = 0;
        repeatSong += channel[0].repeatPoint;
        repeatSong += channel[1].repeatPoint;
        repeatSong += channel[2].repeatPoint;
        repeatSong += channel[3].repeatPoint;

        if (repeatSong != 0)
        {
          tickCount = 0;
          //channelActiveMute = 0xF0;
          //channel[0].ptr = trackAddress((channel[0].repeatPoint)) - (entryBase + 4);
          //channel[1].ptr = trackAddress((channel[1].repeatPoint)) - (entryBase + 4);
          //channel[2].ptr = trackAddress((channel[2].repeatPoint)) - (entryBase + 4);
          //channel[3].ptr = trackAddress((channel[3].repeatPoint)) - (entryBase + 4);

          //channel[0].delay = 0;
          //channel[1].delay = 0;
          //channel[2].delay = 0;
          //channel[3].delay = 0;
        }
      }
    }
    
    // References
    this.id = _id;
    this.jumpTo = _jumpTo;
    this.play = _play;
  }

  // Define playback channels
  var channel = [
    new Channel(0),
    new Channel(1),
    new Channel(2),
    new Channel(3)
  ];

  // == INTERFACE == //
  
  // Called by synth when connecting graph
  // Provides reference to synthesizer
  function _setup(squawkSynth) {
    synth = squawkSynth;
    synth.setTick(sampleRate / 25); // Default to 25 ticks per second 
  };

  // Retrieve tick counter
  function _getTickCount() {
    return tickCount;
  }

  // Retrieve Active and Muted channels
  function _getChannelActiveMute() {
    return (channelActiveMute & 0xF0);
  }

  // Retrieve noRepeat
  function _getNoRepeat() {
    return [noRepeat, entryBase, channel[0].repeatPoint, channel[1].repeatPoint, channel[2].repeatPoint, channel[3].repeatPoint];
  }

  // Called by synthesizer each tick
  function _tick() {
    // Count ticks for no technical reason whatsoever
    tickCount++;
    // Run all channels
    channel.forEach(function(e) { e.play(); });
    // Run oscillator testing function (make insanity sounds)
    // DMEP();
  }

  // Set source of music data to either a callback function or an array
  function _setSource(source) {
    if(typeof source == "function") {
      // Source from callback function (useful for interfacing with tracker).
      // Jumping and looping must be handled by callback function.
      
      data = source;
    } else {
      // Source from array.
      // Mimics playback on embedded device.

      // Auto-fill algorithm (for track addresses)
      var temp = new Array();
      for(var n = 0, a = 0, b = 0; n < source.length; n++) {
        if(typeof source[n] == "string") {
          temp[(a << 1) + 1] = b >> 8;
          temp[(a << 1) + 2] = b & 0xFF;
          a++;
        } else {
          temp.push(source[n]);
          b++;
        }
      }
      
      // Copy to typed array
      data = new Uint8Array(new ArrayBuffer(b));
      for(var n = 0; n < b; n++) data[n] = temp[n];

      // Set up entry tracks for each channel
      entryBase = (readByte(0) << 1) + 1;
      channel.forEach(function(e) { e.jumpTo(trackAddress(readByte(entryBase + e.id()))); });
    }
  }

  // References
  this.setup = _setup;
  this.getTickCount = _getTickCount;
  this.getChannelActiveMute = _getChannelActiveMute;
  this.getNoRepeat = _getNoRepeat;
  this.tick = _tick;
  this.setSource = _setSource;
}