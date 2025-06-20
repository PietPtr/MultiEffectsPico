Driver for a baseboard + devkit hat

# devkit hat board

Board to attach to the baseboard via the GPIO breakout on the side. Contains footprints for:

1) OLED display
2) potentiometers
3) encoders
4) RGB LEDs above each encoder / pot
4) some pushbuttons (kailh silent)
5) some toggle switches (the big ones)
6) (probably) a GPIO expander: Pcf8575

# Interface

Main menu which selects an effect or tool and configures it. Select an item with the encoder, press to continue. Press the back button to go back. Every fixed point setting will be saved in some atomic u32, and a mapping to the encoders and pots can be created with the pushbuttons for every setting. The LEDs will make this user interface clear. The effects can only be applied to the jack input.

## Metronome

- Tempo: 0 to 250 BPM, unsigned, fixed point of any precision
- Volume: 0 to 1, I1F15

## Amplify / Overdrive

- Amplification: U8F8

## Tuner

At some performant interval, tries to figure out the note / deviation on its input and renders it nicely

## EQ

A couple of bands, each with their own mix level, tbd

## Mix

- Aux volume: 0 to 1, I1F15
- Jack volume: 0 to 1, I1F15

# TODO

- Make a version of Amplify called Attenuate that uses I1F15 for more performant volume-setting (since no fixedpoint conversion is necessary)