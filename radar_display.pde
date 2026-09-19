/* =============================================================================
   radar_display.pde  —  PPI-style radar scope for the Arduino sweep radar
   -----------------------------------------------------------------------------
   Reads "angle,distance_cm" lines from the Arduino over USB serial and draws a
   classic plan-position-indicator display: range rings, a rotating sweep line,
   and contacts that fade out over time.

   Keys:  S  save a PNG screenshot into ./screenshots/
          C  clear all contacts
          R  reconnect to the serial port

   Author  : <your name>
   Licence : MIT
   ========================================================================== */

import processing.serial.*;

/* ========================== configuration ============================== */

final int    BAUD         = 115200;   // MUST match Serial.begin() on Arduino
final int    MAX_RANGE_CM = 200;      // MUST match MAX_RANGE_CM on Arduino
final int    ANGLE_MIN    = 15;
final int    ANGLE_MAX    = 165;
final int    TRAIL_MS     = 1500;     // how long a contact stays on screen
final String FONT_NAME    = "Menlo";  // any monospaced font on your Mac

/* Leave empty for automatic port detection. If auto-detect picks the wrong
   device, run the sketch once, copy the correct "/dev/cu.*" name out of the
   console listing, and paste it here. */
final String FORCE_PORT   = "";

/* Set true if left and right come out mirrored on screen relative to reality.
   (Whether they do depends on which way round you taped the sensor to the
   servo horn — flipping this is easier than re-taping.) */
final boolean MIRROR_X    = false;

/* ============================== state ================================== */

Serial  port;
boolean linkUp    = false;
String  portLabel = "—";
int     lastPacketMs = 0;
long    packetCount  = 0;

int angleNow    = ANGLE_MIN;   // where the beam is pointing right now
int distanceNow = 0;           // most recent reading (0 = no contact)
int anglePrev   = ANGLE_MIN;
int sweepDir    = 1;           // +1 sweeping up in angle, -1 sweeping down

/* One slot per whole degree. echoCm holds the range of the last contact seen
   at that bearing; echoStamp records when, so we can fade it out. */
int[] echoCm    = new int[181];
int[] echoStamp = new int[181];

PFont fontBig, fontSmall;
float cx, cy, R;               // scope centre and radius, recomputed per frame

/* ================================ setup ================================ */

void setup() {
  size(1200, 720);
  surface.setResizable(true);
  surface.setTitle("Arduino Ultrasonic Radar");
  smooth(8);

  fontBig   = createFont(FONT_NAME, 26);
  fontSmall = createFont(FONT_NAME, 15);

  openPort();
}

void openPort() {
  linkUp = false;
  if (port != null) { try { port.stop(); } catch (Exception e) { } port = null; }

  String chosen = choosePort();
  if (chosen == null) {
    println("!! No serial port found. Plug in the Arduino, then press R.");
    portLabel = "no port";
    return;
  }

  try {
    port = new Serial(this, chosen, BAUD);
    /* bufferUntil('\n') tells Processing to accumulate incoming bytes and only
       fire serialEvent() once a complete line has arrived. Without it you get
       serialEvent() per byte and have to reassemble lines yourself. */
    port.bufferUntil('\n');
    port.clear();
    linkUp    = true;
    portLabel = chosen;
    println("Connected to " + chosen + " at " + BAUD + " baud.");
  }
  catch (Exception e) {
    /* Almost always means something else already owns the port — usually the
       Arduino IDE's Serial Monitor. Close it and press R. */
    println("!! Could not open " + chosen + ": " + e.getMessage());
    portLabel = "busy: " + chosen;
  }
}

/* Pick the most likely Arduino port. On macOS, prefer the "cu." (call-up)
   devices over "tty." ones: tty.* blocks waiting for a carrier-detect signal
   the Arduino never asserts. */
String choosePort() {
  String[] ports = Serial.list();
  println("--- serial ports visible to Processing ---");
  printArray(ports);
  println("------------------------------------------");

  if (FORCE_PORT.length() > 0) return FORCE_PORT;
  if (ports.length == 0) return null;

  // usbmodem = ATmega16U2 boards, wchusbserial/usbserial = CH340 boards
  String[] hints = { "usbmodem", "wchusbserial", "usbserial", "SLAB_USBtoUART" };
  for (int h = 0; h < hints.length; h++) {
    for (int i = 0; i < ports.length; i++) {
      if (ports[i].indexOf("cu.") >= 0 && ports[i].indexOf(hints[h]) >= 0) {
        return ports[i];
      }
    }
  }
  return ports[0];
}

/* ========================= serial input ================================ */

/* Called automatically by Processing whenever a full '\n'-terminated line has
   been received. Everything here is defensive: a half-line arrives whenever
   you start the sketch mid-transmission, and the Arduino also emits '#'
   comment lines at boot. */
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line);
  if (line.length() == 0)   return;
  if (line.charAt(0) == '#') return;      // comment line from the firmware

  int comma = line.indexOf(',');
  if (comma < 1) return;

  int a, d;
  try {
    a = Integer.parseInt(line.substring(0, comma).trim());
    d = Integer.parseInt(line.substring(comma + 1).trim());
  }
  catch (NumberFormatException e) {
    return;                               // truncated or corrupted line
  }

  if (a < 0 || a > 180 || d < 0) return;

  if (a != anglePrev) {
    sweepDir  = (a > anglePrev) ? 1 : -1;
    anglePrev = a;
  }

  angleNow     = a;
  distanceNow  = d;
  lastPacketMs = millis();
  packetCount++;

  if (d > 0) { echoCm[a] = d; echoStamp[a] = millis(); }
  else       { echoCm[a] = 0; }           // beam swept through: clear it
}

/* ============================== drawing ================================ */

void draw() {
  background(3, 10, 6);

  /* Recomputed every frame so the layout survives window resizing. The scope
     sits at the bottom centre, because we only cover the forward half-plane. */
  cx = width / 2.0;
  cy = height * 0.90;
  R  = min(width * 0.46, height * 0.80);

  drawGrid();
  drawContacts();
  drawSweep();
  drawHud();
}

/* Convert a bearing in degrees and a range in pixels to screen coordinates.
   Screen Y grows downward, so "up" needs a MINUS on the sine term. */
float sx(float deg, float r) {
  float x = r * cos(radians(deg));
  return cx + (MIRROR_X ? -x : x);
}
float sy(float deg, float r) {
  return cy - r * sin(radians(deg));
}

void drawGrid() {
  noFill();
  stroke(0, 190, 95, 110);
  strokeWeight(1.5);

  // four range rings at 25 / 50 / 75 / 100 % of maximum range
  for (int i = 1; i <= 4; i++) {
    float dia = 2 * R * i / 4.0;
    arc(cx, cy, dia, dia, PI, TWO_PI);
  }

  // bearing spokes every 30 degrees, plus the horizon line
  for (int a = 0; a <= 180; a += 30) line(cx, cy, sx(a, R), sy(a, R));
  line(cx - R, cy, cx + R, cy);

  // shade the arc the servo cannot reach, so the blind zone is obvious
  noStroke();
  fill(255, 255, 255, 8);
  arc(cx, cy, 2 * R, 2 * R, PI, PI + radians(ANGLE_MIN));
  arc(cx, cy, 2 * R, 2 * R, TWO_PI - radians(180 - ANGLE_MAX), TWO_PI);

  // labels
  textFont(fontSmall);
  fill(0, 215, 110, 210);
  textAlign(CENTER, CENTER);
  for (int a = 0; a <= 180; a += 30) text(a + "°", sx(a, R + 26), sy(a, R + 26));

  textAlign(LEFT, BOTTOM);
  for (int i = 1; i <= 4; i++) {
    int cm = MAX_RANGE_CM * i / 4;
    text(cm + " cm", cx + 10, cy - R * i / 4.0 - 5);
  }
}

void drawContacts() {
  int now = millis();

  for (int a = 0; a <= 180; a++) {
    int d = echoCm[a];
    if (d <= 0) continue;

    int age = now - echoStamp[a];
    if (age > TRAIL_MS) continue;

    float life = 1.0 - (float) age / TRAIL_MS;         // 1.0 fresh -> 0.0 stale
    float pr   = map(constrain(d, 0, MAX_RANGE_CM), 0, MAX_RANGE_CM, 0, R);
    float px   = sx(a, pr), py = sy(a, pr);

    // the "shadow" the target casts: everything behind it is unobservable
    stroke(255, 70, 70, 85 * life);
    strokeWeight(2);
    line(px, py, sx(a, R), sy(a, R));

    // the contact itself, with a soft halo
    noStroke();
    fill(255, 90, 90, 60 * life);
    ellipse(px, py, 20, 20);
    fill(255, 130, 130, 255 * life);
    ellipse(px, py, 8, 8);
  }
}

void drawSweep() {
  // a short decaying tail behind the beam sells the rotation
  strokeWeight(2);
  for (int i = 20; i >= 1; i--) {
    float a     = angleNow - sweepDir * i * 1.3;
    float alpha = map(i, 1, 20, 85, 0);
    stroke(80, 255, 140, alpha);
    line(cx, cy, sx(a, R), sy(a, R));
  }

  stroke(170, 255, 195, 235);
  strokeWeight(2.5);
  line(cx, cy, sx(angleNow, R), sy(angleNow, R));

  noStroke();
  fill(170, 255, 195, 235);
  ellipse(cx, cy, 9, 9);
}

void drawHud() {
  boolean stale = (millis() - lastPacketMs) > 2000;

  textFont(fontBig);
  textAlign(LEFT, TOP);
  fill(0, 235, 130);
  text("ULTRASONIC RADAR", 26, 22);

  textFont(fontSmall);
  fill(0, 200, 110, 200);
  text("port " + portLabel + "   ·   " + BAUD + " baud   ·   "
       + packetCount + " readings   ·   " + int(frameRate) + " fps", 26, 58);
  text("[S] screenshot   [C] clear   [R] reconnect", 26, 78);

  // live readout, bottom right
  textAlign(RIGHT, BOTTOM);
  textFont(fontBig);
  fill(0, 235, 130);
  text("BEARING  " + nf(angleNow, 3) + "°", width - 26, height - 54);
  if (distanceNow > 0) {
    fill(255, 130, 130);
    text("RANGE    " + nf(distanceNow, 3) + " cm", width - 26, height - 20);
  } else {
    fill(0, 150, 90);
    text("RANGE    no contact", width - 26, height - 20);
  }

  if (!linkUp || stale) {
    textAlign(CENTER, CENTER);
    textFont(fontBig);
    fill(255, 200, 60);
    String msg = linkUp ? "NO DATA — is the Arduino still running?"
                        : "NOT CONNECTED — close the Serial Monitor, then press R";
    text(msg, width / 2.0, height * 0.42);
  }
}

/* =============================== input ================================= */

void keyPressed() {
  if (key == 's' || key == 'S') {
    saveFrame("screenshots/radar-####.png");
    println("screenshot saved into the sketch folder");
  }
  if (key == 'c' || key == 'C') {
    for (int i = 0; i <= 180; i++) { echoCm[i] = 0; echoStamp[i] = 0; }
  }
  if (key == 'r' || key == 'R') openPort();
}
