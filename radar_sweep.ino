/* =============================================================================
   radar_sweep.ino  —  Ultrasonic sweep radar for the Arduino UNO R3
   -----------------------------------------------------------------------------
   Hardware : Elegoo UNO R3 + HC-SR04 ultrasonic sensor + SG90 micro servo
   Purpose  : Rotate the sensor through a fixed arc, measure the distance to the
              nearest object at each angle, and stream the readings over USB
              serial for the companion Processing sketch to plot.

   SERIAL PROTOCOL (one measurement per line, terminated with '\n')

       <angle>,<distance_cm>\n

       angle        integer degrees, 0..180
       distance_cm  integer centimetres, or 0 meaning "no echo / out of range"

   Any line beginning with '#' is a human-readable comment and is ignored by
   the Processing client. Using a plain newline (rather than an exotic
   terminator) means you can also just open Tools > Serial Monitor and read the
   stream yourself while debugging.

   Author  : <your name>
   Licence : MIT
   ========================================================================== */

#include <Servo.h>

/* ----------------------------------------------------------------- pin map
   The Servo library uses Timer1 on the UNO, which disables analogWrite() on
   pins 9 and 10. We only ever use these pins digitally, so that is harmless.
   The servo signal itself can live on ANY digital pin — the library bit-bangs
   its own 50 Hz pulse train rather than relying on hardware PWM.            */
const uint8_t TRIG_PIN  = 10;   // HC-SR04 Trig  (output from Arduino)
const uint8_t ECHO_PIN  = 11;   // HC-SR04 Echo  (input  to  Arduino)
const uint8_t SERVO_PIN = 12;   // SG90 signal   (orange / yellow wire)

/* ------------------------------------------------------------- sweep shape
   SG90 servos are nominally 0..180 degrees but most of them bind, buzz or
   stall in the last few degrees at each end. Staying inside 15..165 keeps the
   gearbox happy and stops the servo drawing a stall current that can brown
   out the whole board.                                                      */
const int ANGLE_MIN  = 15;      // degrees
const int ANGLE_MAX  = 165;     // degrees
const int ANGLE_STEP = 1;       // degrees per measurement (raise for speed)

/* ----------------------------------------------------------------- timing
   SETTLE_MS is the single most important tuning knob. The servo needs time to
   physically arrive at the commanded angle; if you ping while the head is
   still swinging, the echo comes back from wherever the sensor happened to be
   pointing and the picture smears. Too long and the sweep crawls.

   ECHO_TIMEOUT_US caps how long pulseIn() waits for the echo pulse. This is
   what makes "nothing there" fast instead of costing a full second.
       15000 us of flight time  ->  15000 * 0.0343 / 2  ~=  257 cm
   which comfortably covers our 200 cm display range.                        */
const unsigned int  SETTLE_MS       = 15;
const unsigned long ECHO_TIMEOUT_US = 15000UL;

/* Number of pings per angle. 1 is fast and responsive. Set to 3 (or 5) to take
   the MEDIAN of several pings, which throws away the occasional wild reading
   the HC-SR04 produces when it catches a stray reflection — at the cost of a
   proportionally slower sweep. Must be an odd number.                       */
const uint8_t      SAMPLES       = 1;
const unsigned int INTER_PING_MS = 6;   // settling gap between repeat pings

/* ------------------------------------------------------------------ range
   The datasheet claims 2 cm to 400 cm. In practice anything past ~2 m off a
   small or soft target is unreliable, so we clamp to a range we can trust and
   report everything else as 0 ("no contact").                               */
const int MAX_RANGE_CM = 200;
const int MIN_RANGE_CM = 2;

/* Speed of sound is 343 m/s at 20 C, which is 0.0343 cm per microsecond.
   It varies with temperature by roughly 0.17% per degree C — see the
   "Calibration" section of the build guide if you want to compensate.       */
const float CM_PER_US = 0.0343f;

Servo radarHead;

/* ========================================================================= */

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);      // make sure we start from a known state

  radarHead.attach(SERVO_PIN);
  radarHead.write(ANGLE_MIN);
  delay(600);                       // let the head travel to the start angle

  // F() keeps this string in flash instead of burning precious SRAM.
  Serial.println(F("# radar v1.0 | format: angle,distance_cm | 0 = no echo"));
}

void loop() {
  // Out-and-back. Measuring on both legs doubles the effective refresh rate
  // compared with flying back to the start with the sensor switched off.
  for (int a = ANGLE_MIN; a <= ANGLE_MAX; a += ANGLE_STEP) measureAt(a);
  for (int a = ANGLE_MAX; a >= ANGLE_MIN; a -= ANGLE_STEP) measureAt(a);
}

/* ------------------------------------------------------------------------ */

/* Point the head at `angle`, take a reading, and report it. */
void measureAt(int angle) {
  radarHead.write(angle);
  delay(SETTLE_MS);

  int cm = readDistanceCm();

  Serial.print(angle);
  Serial.print(',');
  Serial.println(cm);               // println supplies the '\n' terminator
}

/* Take SAMPLES pings and return the median, which rejects single-ping
   outliers far better than an average does (one absurd 400 cm reading drags
   an average badly, but barely moves a median).                             */
int readDistanceCm() {
  int s[SAMPLES];

  for (uint8_t i = 0; i < SAMPLES; i++) {
    s[i] = pingOnceCm();
    if (i + 1 < SAMPLES) delay(INTER_PING_MS);
  }

  // Insertion sort. With 1-5 elements this is faster than anything clever.
  for (uint8_t i = 1; i < SAMPLES; i++) {
    int v = s[i];
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) { s[j] = s[j - 1]; j--; }
    s[j] = v;
  }

  return s[SAMPLES / 2];
}

/* One time-of-flight measurement.
   Returns centimetres, or 0 for "no usable echo".                           */
int pingOnceCm() {
  /* The HC-SR04 is triggered by a 10 us HIGH pulse on Trig. It then emits
     eight 40 kHz bursts and raises Echo. Echo stays HIGH for exactly as long
     as the sound is in flight, so the width of that pulse IS the measurement. */
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);             // clean LOW baseline before the pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);            // the datasheet-specified trigger width
  digitalWrite(TRIG_PIN, LOW);

  /* pulseIn() blocks until Echo goes HIGH then LOW again, and returns the
     HIGH duration in microseconds. It returns 0 if the timeout expires. */
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return 0;            // timed out: nothing within range

  /* The sound travelled OUT to the target and BACK again, so the one-way
     distance is half the total path:  d = (t * v) / 2                        */
  int cm = (int)((us * CM_PER_US) * 0.5f);

  if (cm < MIN_RANGE_CM || cm > MAX_RANGE_CM) return 0;
  return cm;
}