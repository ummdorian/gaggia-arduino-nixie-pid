// ArduiNIX: D2-D5 = cathode bank 1 (BCD A..D), D6-D9 = bank 2, D10-D13 = anode bank.

const uint8_t BANK1[4] = {2, 3, 4, 5};
const uint8_t BANK2[4] = {6, 7, 8, 9};
// Anode bank is silkscreened opposite the stock ArduiNIX sketches: D13 is anode 1.
const uint8_t ANODE[4] = {13, 12, 11, 10};

// K155ID1 drives no output for BCD > 9, so 15 blanks a tube.
const uint8_t BLANK = 15;

// All three tubes share cathode bank 1; only the anode differs. Left, middle, right.
const uint8_t TUBE_ANODE[3] = {1, 2, 3};

const uint8_t PIN_DIAL   = A0;
const uint8_t PIN_BUTTON = A1;
const uint8_t PIN_SSR    = A2;
const uint8_t PIN_RTD    = A3;

const uint8_t DIAL_BITS = 2;
const uint8_t RTD_BITS = 4;
const uint16_t RTD_FS = 1023u << RTD_BITS;
const float R_REF = 982.0;  // measured, not the nominal 1k
const uint8_t DIAL_AVG_SHIFT = 4;

// Dial is a non-linear taper, so map by measured breakpoints rather than a formula.
// 88-90 sit within four raw ADC codes of each other and are not reliably separable.
struct DialPoint { float adc; uint8_t tempC; };
const DialPoint DIAL_CAL[] = {
  {   1.9, 88},
  {   2, 89},
  {  10, 90},
  {  82, 91},
  { 152, 92},
  { 224, 93},
  { 306, 94},
  { 381, 95},
  { 431, 96},
};
const uint8_t DIAL_CAL_N = sizeof(DIAL_CAL) / sizeof(DIAL_CAL[0]);

// The probe reads boiler wall, not brew water. Published drops cluster at 8-12 C
// (GaggiMate ships 8, Papel Espresso 8-12, Home-Barista reports 5-14). Measure yours.
const float BREW_OFFSET_C = 10.0;

// PID gains, output in percent power. Starting points only - tune on the machine.
const float GAIN_PROPORTIONAL = 15.0;  // % per degree of error: the immediate response
const float GAIN_INTEGRAL = 0.20;      // % per second per degree: erases steady offset
const float GAIN_DERIVATIVE = 40.0;    // % per degree-per-second of rise: brakes overshoot

const uint32_t WINDOW_MS = 1000;  // zero-cross SSR needs time-proportional, not PWM
const uint16_t DIAL_MS = 100;     // faster than the PID window so the held display tracks the knob

// Out-of-range counts: ~80 ohm and ~250 ohm, either side of anything physical.
const uint16_t RTD_MIN_COUNT = 1233;
const uint16_t RTD_MAX_COUNT = 3321;
const float RUNAWAY_RISE_C = 2.0;
const uint32_t RUNAWAY_MS = 300000UL;   // full power this long without rising = fault

const uint32_t WAKE_MS = 300000UL;      // 5 min
const uint16_t BUTTON_GRACE_MS = 400;   // contact is unreliable; bridge brief dropouts
const uint32_t DEPOISON_STEP_MS = 5000; // datasheet 4.8 minimum per cathode

// Code 3 is retired: steam mode legitimately exceeds any brew-safe ceiling, so an
// over-temperature fault cannot be distinguished from normal steaming.
enum Fault : uint8_t {
  FAULT_NONE = 0,
  FAULT_PROBE_OPEN = 1,
  FAULT_PROBE_SHORT = 2,
  FAULT_RUNAWAY = 4,
};

volatile uint8_t displayDigits[3] = {BLANK, BLANK, BLANK};
volatile bool displayLit = false;

Fault fault = FAULT_NONE;

static void writeBank(const uint8_t pins[4], uint8_t bcd) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(pins[i], (bcd >> i) & 1);
  }
}

static void anodesOff() {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(ANODE[i], LOW);
  }
}

// One tube per interrupt. The dozen digitalWrites between anodes-off and anode-on are
// themselves the ~60 us deionisation gap, so no explicit delay is needed here.
ISR(TIMER2_COMPA_vect) {
  static uint8_t slot = 0;

  anodesOff();
  if (!displayLit) {
    return;
  }
  slot = (slot + 1) % 3;
  writeBank(BANK1, displayDigits[slot]);
  digitalWrite(ANODE[TUBE_ANODE[slot] - 1], HIGH);
}

static void setDisplay(int value) {
  if (value < 0) value = 0;
  if (value > 999) value = 999;
  uint8_t hundreds = value / 100;
  displayDigits[0] = hundreds ? hundreds : BLANK;
  displayDigits[1] = (value / 10) % 10;
  displayDigits[2] = value % 10;
}

static void setDisplayAll(uint8_t digit) {
  displayDigits[0] = digit;
  displayDigits[1] = digit;
  displayDigits[2] = digit;
}

// 4^n samples summed then shifted by n yields n extra bits; the rig's own switching
// noise supplies the dither this needs.
static uint16_t readOversampled(uint8_t pin, uint8_t extraBits) {
  uint16_t samples = 1 << (2 * extraBits);
  uint32_t acc = 0;
  for (uint16_t i = 0; i < samples; i++) {
    acc += analogRead(pin);
  }
  return acc >> extraBits;
}

static uint8_t dialTemperature(float adc) {
  uint8_t best = 0;
  float bestDist = fabs(adc - DIAL_CAL[0].adc);
  for (uint8_t i = 1; i < DIAL_CAL_N; i++) {
    float d = fabs(adc - DIAL_CAL[i].adc);
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  return DIAL_CAL[best].tempC;
}

// Linear PT100: exact at 0 and 100 C, about 0.4 C low near 50 C.
static float rtdCelsius(float ohms) {
  return (ohms / 100.0 - 1.0) / 0.00385;
}

static float pidOutput(float setpoint, float temp, float dtSec) {
  static float integral = 0;
  static float lastTemp = NAN;

  float error = setpoint - temp;

  // Accumulate already scaled so retuning the integral gain never steps the output.
  integral += GAIN_INTEGRAL * error * dtSec;
  if (integral > 100.0) integral = 100.0;
  if (integral < 0.0) integral = 0.0;

  // Derivative on measurement, not error, so moving the dial cannot kick the output.
  float derivative = 0;
  if (!isnan(lastTemp)) {
    derivative = -GAIN_DERIVATIVE * (temp - lastTemp) / dtSec;
  }
  lastTemp = temp;

  float out = GAIN_PROPORTIONAL * error + integral + derivative;
  if (out > 100.0) out = 100.0;
  if (out < 0.0) out = 0.0;
  return out;
}

void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(BANK1[i], OUTPUT);
    pinMode(BANK2[i], OUTPUT);
    pinMode(ANODE[i], OUTPUT);
  }
  anodesOff();
  writeBank(BANK2, BLANK);  // bank 2 is unused and stays blanked

  // Default AVCC reference throughout: the PT100 divider is ratiometric, so supply
  // drift cancels. Switching to INTERNAL would gain resolution and lose that.
  pinMode(PIN_DIAL, INPUT);
  pinMode(PIN_RTD, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_SSR, LOW);  // LOW before OUTPUT so the heater cannot glitch on at boot
  pinMode(PIN_SSR, OUTPUT);

  // Timer2 CTC, /1024, OCR 31 -> 2.048 ms per tube, 163 Hz refresh.
  // No watchdog: the Duemilanove's ATmegaBOOT does not clear WDRF and would boot-loop.
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  OCR2A = 31;
  TIMSK2 = (1 << OCIE2A);

  Serial.begin(115200);
}

void loop() {
  uint32_t now = millis();

  static uint32_t lastLowMs = 0;
  static bool everLow = false;
  static bool wasHeld = false;
  static uint32_t wakeUntil = 0;

  static uint32_t windowStart = 0;
  static bool windowPrimed = false;
  static float output = 0;
  static uint32_t onMs = 0;

  static uint8_t brewSetpoint = 0;
  static float temp = NAN;
  static int32_t dialAcc = -1;
  static float dialAvg = 0;

  static uint8_t depoisonDigit = 0;
  static uint32_t depoisonStepAt = 0;
  static bool depoisonDone = false;

  static bool reachedSetpoint = false;
  static bool wasWakeActive = false;
  static uint32_t fullPowerSince = 0;
  static float fullPowerTemp = 0;
  static bool atFullPower = false;

  if (digitalRead(PIN_BUTTON) == LOW) {
    lastLowMs = now;
    everLow = true;
  }
  bool held = everLow && (now - lastLowMs < BUTTON_GRACE_MS);
  if (held && !wasHeld) {
    wakeUntil = now + WAKE_MS;
    depoisonDone = true;  // a press wants the display, not the cycle; it reruns at session end
  }
  wasHeld = held;

  // Cathodes only poison while a narrow digit set is lit, so exercise them as the
  // display goes dark rather than on a clock.
  bool wakeActive = !reachedSetpoint || (int32_t)(wakeUntil - now) > 0;
  if (wasWakeActive && !wakeActive) {
    depoisonDone = false;
    depoisonDigit = 0;
    depoisonStepAt = now;
  }
  wasWakeActive = wakeActive;

  if (!depoisonDone && now - depoisonStepAt >= DEPOISON_STEP_MS) {
    depoisonStepAt += DEPOISON_STEP_MS;
    if (depoisonDigit >= 9) {
      depoisonDone = true;
    } else {
      depoisonDigit++;
    }
  }

  static uint32_t lastDialMs = 0;
  if (dialAcc < 0 || now - lastDialMs >= DIAL_MS) {
    lastDialMs = now;
    uint16_t dialRaw = readOversampled(PIN_DIAL, DIAL_BITS);
    if (dialAcc < 0) {
      dialAcc = (int32_t)dialRaw << DIAL_AVG_SHIFT;
    } else {
      dialAcc += (int32_t)dialRaw - (dialAcc >> DIAL_AVG_SHIFT);
    }
    dialAvg = dialAcc / (float)(1 << DIAL_AVG_SHIFT);
    brewSetpoint = dialTemperature(dialAvg);
  }

  if (!windowPrimed || now - windowStart >= WINDOW_MS) {
    if (!windowPrimed) {
      windowStart = now;
      windowPrimed = true;
    } else {
      windowStart += WINDOW_MS;
    }

    uint16_t rtd = readOversampled(PIN_RTD, RTD_BITS);
    temp = rtdCelsius(R_REF * rtd / (float)(RTD_FS - rtd));

    if (fault == FAULT_NONE) {
      if (rtd >= RTD_MAX_COUNT) {
        fault = FAULT_PROBE_OPEN;
      } else if (rtd <= RTD_MIN_COUNT) {
        fault = FAULT_PROBE_SHORT;
      }
    }

    float boilerSetpoint = brewSetpoint + BREW_OFFSET_C;
    output = (fault == FAULT_NONE) ? pidOutput(boilerSetpoint, temp, WINDOW_MS / 1000.0) : 0;
    onMs = (uint32_t)(output * WINDOW_MS / 100.0);

    if (output >= 99.9) {
      if (!atFullPower) {
        atFullPower = true;
        fullPowerSince = now;
        fullPowerTemp = temp;
      } else if (now - fullPowerSince >= RUNAWAY_MS && temp - fullPowerTemp < RUNAWAY_RISE_C) {
        fault = FAULT_RUNAWAY;
      }
    } else {
      atFullPower = false;
    }

    if (!reachedSetpoint && temp >= boilerSetpoint) {
      reachedSetpoint = true;
      wakeUntil = now + WAKE_MS;
    }

    Serial.print(F("set "));
    Serial.print(brewSetpoint);
    Serial.print(F("  boiler "));
    Serial.print(boilerSetpoint, 1);
    Serial.print(F("  temp "));
    Serial.print(temp, 1);
    Serial.print(F("  out "));
    Serial.print(output, 1);
    Serial.print(F("  dial "));
    Serial.print(dialAvg, 2);
    Serial.print(F("  fault "));
    Serial.println(fault);
  }

  digitalWrite(PIN_SSR, (fault == FAULT_NONE && now - windowStart < onMs) ? HIGH : LOW);

  if (fault != FAULT_NONE) {
    setDisplayAll(fault);
    displayLit = true;
  } else if (held) {
    setDisplay(brewSetpoint);
    displayLit = true;
  } else if (!depoisonDone) {
    setDisplayAll(depoisonDigit);
    displayLit = true;
  } else if (wakeActive) {
    setDisplay((int)(isnan(temp) ? 0 : temp - BREW_OFFSET_C + 0.5));
    displayLit = true;
  } else {
    displayLit = false;
  }
}
