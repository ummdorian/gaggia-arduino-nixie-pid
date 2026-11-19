# gaggia-arduino-nixie-pid

PID temperature control for a Gaggia Classic Pro, displayed on three IN-16 nixie tubes.

- Arduino (ATmega328P) + ArduiNIX shield. All three tubes share one cathode bank, multiplexed from Timer2.
- PT100 read directly by the ADC against a 982 Ω reference, oversampled to 14 bits. No MAX31865.
- Time-proportional SSR output on a 1 s window. Dial sets 88–96 °C; the tubes show estimated brew temperature.
- Display dark by default, wakes 5 minutes on the button, cycles every digit at end of session to fight cathode poisoning.
- Fault codes on the tubes: 1 probe open, 2 probe short, 4 runaway.

Each tube needs its own anode ballast resistor. The ArduiNIX has none, and paralleled tubes current-hog.

Mains voltage and ~190 V DC. Build at your own risk.
