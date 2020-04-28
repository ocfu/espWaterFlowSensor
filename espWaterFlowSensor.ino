
// needed for the os timer
extern "C" {
#include "user_interface.h"
}

// GPIO ports and states
const int led = D0;         // indicates, if waterflow is above minimum

const int pulseInput = D3;  // input of the pulse sensor
const int pulseOutput = D2; // test signal output



// Debounced input signal
//
//   ^
//   |
//  1|       ++ +----+          +----+++
//   |       || |    |          |    |||
//   |       || |    |          |    |||
//  0+----S--++-+-S--+----S-----+-S--+++--S-> t
//
//
//   ^
//   |
//  1|          +----+          +----+
//   |          |    |          |    |
//   |          |    |          |    |
//  0+----S-----+-S--+----S-----+-S--+----S-> t
//              /<-->/<-------->/
//                tp      t0
//              /<------------->/ /<----->/
//                      T            Ts
//
// Pulse rate (freqency) f = 1/T
//
// Within the period T we need to detect (sample, S) a pulse high state and a
// pulse low state. Theat means we need to read at least twice the input within T.
//
// Sample Frequency  fs = 2f
// Sample Period     Ts =  T/2
//
// Example
//
// The max. expected pulse rate (frequency) is fmx = 50Hz (50 pulses/s)
// T = 1/f = 0.02s = 20ms
// Ts = T/2 = 10ms
//
// The time 0-10ms we can use for the debounce.
//
// Application: Waterflow sensor
//
// Sensor properties:
//
//  Pmax:       17.5 bar               Max. Pressure
//  Range:      2...100l/min           Flow range min...max
//  Iinmax:     15mA                   Current power supply
//  Vcc:        5V                     Voltage power supply
//  Ioutmax:    10mA                   Max. Current at output (pulsed signal)
//  Precision:  >1%
//  Pulse/flow: Q(f) = f * 60/K   | [K] = pulses/l converts frequency [f] = 1/s to flow [Q] = l/min
//
float Q = 0.0;                   // measured flow in l/min
const float Qmax = 100.0;        // max. flow l/min
const float Qmin = 2.0;          // min. flow l/min
const float K = 60;              // constant in pulse/l
const float fmx = Qmax * K / 60; // max. frequency (pulses) 1/s (Hz)

float T = 1 / fmx * 1000.0; // T in ms
float Ts = T / 2.0;         // sample time in [ms]

unsigned long pulse_counter = 0;

unsigned long last_sample = 0;  // time in [ms]

ICACHE_RAM_ATTR void isrPulseCounter() {
  // counting the pulses and debounce with within sample period Ts
  unsigned long now = millis();
  if ((now - last_sample) > (unsigned long)round(Ts)) {
    pulse_counter++;
    last_sample = now;
  }
}

// Timer and timer callback. The timer will be called each <timer_rate> seconds and updates 
// the current water flow.
os_timer_t timer1;

int timer_rate = 10;    // call of the timer in s (update rate)

void timerCallback(void *pArg) {
  // Timer to calculate the water flow Q in l/min
  // Q = f / K * 60   | where the factor 60 converts from l/s to l/min
  Q = (float)pulse_counter / (float)timer_rate / K * 60;

  pulse_counter = 0;  // start counting pulses again.
}

// create a test signal
float ft = 21.0;             // test frequency (pulse rate) in 1/s
float Tf = 1 / ft * 1000.0;  // test period in ms 
os_timer_t timer2;

void timerTestPulse(void *pArg) {
  // generates test pulse pattern
  digitalWrite(pulseOutput, !digitalRead(pulseOutput));
}


void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(pulseInput, INPUT_PULLUP);
  pinMode(pulseOutput, OUTPUT);
  digitalWrite(pulseOutput, LOW);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(pulseInput), isrPulseCounter, RISING);
  os_timer_setfn(&timer1, timerCallback, NULL);
  os_timer_arm(&timer1, timer_rate * 1000, true);

  os_timer_setfn(&timer2, timerTestPulse, NULL);
  os_timer_arm(&timer2, (unsigned long)round(Tf / 2.0), true);  // simple 1:1 tp/t0 rate in one function, need to be called 2x faster for the toggle

}

void loop() {
  if (Q >= 2.0)
    digitalWrite(led, HIGH);
  else
    digitalWrite (led, LOW);
  Serial.println(Q);
  delay(1000);
}
