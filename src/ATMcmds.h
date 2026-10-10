#include <stdint.h>

#ifndef ATMCMDS_H
#define ATMCMDS_H

// let's define command list for the ATMlib, that might be easier to use the AMT library

// Let's define the basic commands
#define ATM_DELAY(delay) 				    		0x9F+(delay)								// uses 1 byte
#define ATM_LONG_DELAY(ticks)			    	0xE0,((ticks)-65)							// uses 2 bytes: 65 to 192 ticks (longer: chain delays)
#define ATM_GOTO(onceGoto)				    	0xFC,(onceGoto)							// uses 2 bytes
#define ATM_REPEAT(repeatTimes,track)	  0xFD,(repeatTimes),(track)	// uses 3 bytes
#define ATM_RETURN						    			0xFE												// uses 1 byte

#define ATM_ADD_TEMPO(amount)						0x9C,(amount)								// uses 2 bytes
#define ATM_SET_TEMPO(amount)						0x9D,(amount)								// uses 2 bytes
#define ATM_GOTO_ADV(ch0,ch1,ch2,ch3)		0x9E,(ch0),(ch1),(ch2),(ch3)// uses 5 bytes
#define ATM_STOP_CHAN										0x9F												// uses 1 byte


// Let's define the effect commands
// VOLUME - VOLUME SLIDE
#define ATM_VOL(setVolume)							0x40,(setVolume)						// uses 2 bytes
#define ATM_SL_VOL(slideVolume)					0x41,(slideVolume)					// uses 2 bytes
#define ATM_SL_VOL_ADV(amount,ticks)		0x42,(amount),(ticks)				// uses 3 bytes
#define ATM_SL_VOL_OFF									0x43												// uses 1 byte

// FREQUENCE SLIDE
#define ATM_SL_FRQ(slideFrequency)			0x44,(slideFrequency)				// uses 2 bytes
#define ATM_SL_FRQ_ADV(amount,ticks)		0x45,(amount),(ticks)				// uses 3 bytes
#define ATM_SL_FRQ_OFF									0x46												// uses 1 byte

// ARPEGGIO
#define ATM_ARP(thirdNote,ticks)				0x47,(thirdNote),(ticks)		// uses 3 bytes
#define ATM_ARP_OFF											0x48												// uses 1 byte

// RETRIGGERING NOISE ON THE NOISE CHANNEL 3
#define ATM_NOISE(entryPointAndSpeed)		0x49,(entryPointAndSpeed)		// uses 2 bytes
#define ATM_NOISE_OFF										0x4A												// uses 1 byte

// TRANSPOSITION
#define ATM_ADD_TRA(amount)							0x4B,(amount)								// uses 2 bytes
#define ATM_SET_TRA(amount)							0x4C,(amount)								// uses 2 bytes
#define ATM_TRA_OFF											0x4D												// uses 1 byte

// TREMOLO
#define ATM_TREM(depth,rate)						0x4E,(depth),(rate)					// uses 3 bytes
#define ATM_TREM_OFF										0x4F												// uses 1 byte

// VIBRATO
#define ATM_VIB(depth,rate)							0x50,(depth),(rate)					// uses 3 bytes
#define ATM_VIB_OFF											0x51												// uses 1 byte

// GLISSANDO
#define ATM_GLIS(noteTicks)							0x52,(noteTicks)						// uses 2 bytes
#define ATM_GLIS_OFF										0x53												// uses 1 byte

// NOTE CUT
#define ATM_CUT(amount)									0x54,(amount)								// uses 2 bytes
#define ATM_CUT_OFF											0x55												// uses 1 byte

// Signal the sketch: ATM.check() returns this byte until the next cue or check()
#define ATM_CUE(value)									0x57,(value)								// uses 2 bytes


// let's Define all 64 NOTES from C2 up to D7, actually 63 because note 0 means mute or no note
#define ATM_NOTE_C2       							0x00 + 1  									// uses 1 byte
#define ATM_NOTE_C2_      							0x00 + 2  									// uses 1 byte
#define ATM_NOTE_D2       							0x00 + 3  									// uses 1 byte
#define ATM_NOTE_D2_      							0x00 + 4  									// uses 1 byte
#define ATM_NOTE_E2       							0x00 + 5  									// uses 1 byte
#define ATM_NOTE_F2       							0x00 + 6  									// uses 1 byte
#define ATM_NOTE_F2_      							0x00 + 7  									// uses 1 byte
#define ATM_NOTE_G2       							0x00 + 8  									// uses 1 byte
#define ATM_NOTE_G2_      							0x00 + 9  									// uses 1 byte
#define ATM_NOTE_A2       							0x00 + 10  									// uses 1 byte
#define ATM_NOTE_A2_      							0x00 + 11 									// uses 1 byte
#define ATM_NOTE_B2       							0x00 + 12 									// uses 1 byte
		 
#define ATM_NOTE_C3       						  0x00 + 13 									// uses 1 byte
#define ATM_NOTE_C3_      						  0x00 + 14 									// uses 1 byte
#define ATM_NOTE_D3       						  0x00 + 15 									// uses 1 byte
#define ATM_NOTE_D3_      						  0x00 + 16 									// uses 1 byte
#define ATM_NOTE_E3       						  0x00 + 17 									// uses 1 byte
#define ATM_NOTE_F3       						  0x00 + 18 									// uses 1 byte
#define ATM_NOTE_F3_      						  0x00 + 19 									// uses 1 byte
#define ATM_NOTE_G3       						  0x00 + 20 									// uses 1 byte
#define ATM_NOTE_G3_      						  0x00 + 21 									// uses 1 byte
#define ATM_NOTE_A3       						  0x00 + 22 									// uses 1 byte
#define ATM_NOTE_A3_      						  0x00 + 23 									// uses 1 byte
#define ATM_NOTE_B3       						  0x00 + 24 									// uses 1 byte
		 
#define ATM_NOTE_C4       		 					0x00 + 25 									// uses 1 byte
#define ATM_NOTE_C4_      		 					0x00 + 26 									// uses 1 byte
#define ATM_NOTE_D4       		 					0x00 + 27 									// uses 1 byte
#define ATM_NOTE_D4_      		 					0x00 + 28 									// uses 1 byte
#define ATM_NOTE_E4       		 					0x00 + 29 									// uses 1 byte
#define ATM_NOTE_F4       		 					0x00 + 30 									// uses 1 byte
#define ATM_NOTE_F4_      		 					0x00 + 31 									// uses 1 byte
#define ATM_NOTE_G4       		 					0x00 + 32 									// uses 1 byte
#define ATM_NOTE_G4_      		 					0x00 + 33 									// uses 1 byte
#define ATM_NOTE_A4       		 					0x00 + 34 									// uses 1 byte
#define ATM_NOTE_A4_      		 					0x00 + 35 									// uses 1 byte
#define ATM_NOTE_B4       		 					0x00 + 36 									// uses 1 byte
		 
#define ATM_NOTE_C5       		 					0x00 + 37 									// uses 1 byte
#define ATM_NOTE_C5_      		 					0x00 + 38 									// uses 1 byte
#define ATM_NOTE_D5       		 					0x00 + 39 									// uses 1 byte
#define ATM_NOTE_D5_      		 					0x00 + 40 									// uses 1 byte
#define ATM_NOTE_E5       		 					0x00 + 41 									// uses 1 byte
#define ATM_NOTE_F5       		 					0x00 + 42 									// uses 1 byte
#define ATM_NOTE_F5_      		 					0x00 + 43 									// uses 1 byte
#define ATM_NOTE_G5       		 					0x00 + 44 									// uses 1 byte
#define ATM_NOTE_G5_      		 					0x00 + 45 									// uses 1 byte
#define ATM_NOTE_A5       		 					0x00 + 46 									// uses 1 byte
#define ATM_NOTE_A5_      		 					0x00 + 47 									// uses 1 byte
#define ATM_NOTE_B5       		 					0x00 + 48 									// uses 1 byte
		 
#define ATM_NOTE_C6       		 					0x00 + 49 									// uses 1 byte
#define ATM_NOTE_C6_      		 					0x00 + 50 									// uses 1 byte
#define ATM_NOTE_D6       		 					0x00 + 51 									// uses 1 byte
#define ATM_NOTE_D6_      		 					0x00 + 52 									// uses 1 byte
#define ATM_NOTE_E6       		 					0x00 + 53 									// uses 1 byte
#define ATM_NOTE_F6       		 					0x00 + 54 									// uses 1 byte
#define ATM_NOTE_F6_      		 					0x00 + 55 									// uses 1 byte
#define ATM_NOTE_G6       		 					0x00 + 56 									// uses 1 byte
#define ATM_NOTE_G6_      		 					0x00 + 57 									// uses 1 byte
#define ATM_NOTE_A6       		 					0x00 + 58 									// uses 1 byte
#define ATM_NOTE_A6_      		 					0x00 + 59 									// uses 1 byte
#define ATM_NOTE_B6       		 					0x00 + 60 									// uses 1 byte
		 
#define ATM_NOTE_C7       		 					0x00 + 61 									// uses 1 byte
#define ATM_NOTE_C7_      		 					0x00 + 62 									// uses 1 byte
#define ATM_NOTE_D7       		 					0x00 + 63 									// uses 1 byte



// One-track sound effects for ATMsynth::playSfx(track, ch)
// Example: ATM_SFX_TRACK(sfxJump, ATM_VOL(48), ATM_NOTE_C5, ATM_DELAY(8));
#ifndef ATM_SFX_TRACK
#define ATM_SFX_TRACK(name, ...) \
  const uint8_t name[] PROGMEM = { __VA_ARGS__, ATM_STOP_CHAN }
#endif

#endif
