#include <Servo.h>

const uint8_t TRIG_PIN = 10;
const uint8_t ECHO_PIN = 11;
const uint8_t SERVO_PIN = 12;

const int ANGLE_MIN = 15;
const int ANGLE_MAX = 165;
const int ANGLE_STEP = 1;

const unsigned int SETTLE_MS = 15;
const unsigned long ECHO_TIMEOUT_US = 15000UL;

const uint8_t SAMPLES = 1;
const unsigned int INTER_PING_MS = 6;

const int MAX_RANGE_CM = 200;
const int MIN_RANGE_CM = 2;

const float CM_PER_US = 0.0343f;

Servo radarHead;

void setup()
{
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  radarHead.attach(SERVO_PIN);
  radarHead.write(ANGLE_MIN);
  delay(600);

  Serial.println(F("# radar v1.0 | format: angle,distance_cm | 0 = no echo"));
}

void loop()
{
  for (int a = ANGLE_MIN; a <= ANGLE_MAX; a += ANGLE_STEP) measureAt(a);
  for (int a = ANGLE_MAX; a >= ANGLE_MIN; a -= ANGLE_STEP) measureAt(a);
}

void measureAt(int angle)
{
  radarHead.write(angle);
  delay(SETTLE_MS);

  int cm = readDistanceCm();

  Serial.print(angle);
  Serial.print(',');
  Serial.println(cm);
}

int readDistanceCm()
{
  int s[SAMPLES];

  for (uint8_t i = 0; i < SAMPLES; i++)
  {
    s[i] = pingOnceCm();
    if (i + 1 < SAMPLES) delay(INTER_PING_MS);
  }

  for (uint8_t i = 1; i < SAMPLES; i++)
  {
    int v = s[i];
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) { s[j] = s[j - 1]; j--; }
    s[j] = v;
  }

  return s[SAMPLES / 2];
}

int pingOnceCm()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return 0;

  int cm = (int)((us * CM_PER_US) * 0.5f);

  if (cm < MIN_RANGE_CM || cm > MAX_RANGE_CM) return 0;
  return cm;
}