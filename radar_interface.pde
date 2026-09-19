import processing.serial.*;

final int BAUD = 115200;
final int MAX_RANGE_CM = 200;
final int ANGLE_MIN = 15;
final int ANGLE_MAX = 165;
final int TRAIL_MS = 1500;
final String FONT_NAME = "Menlo";

final String FORCE_PORT = "/dev/cu.usbmodem11201";

final boolean MIRROR_X = false;

Serial  port;
boolean linkUp = false;
String  portLabel = "—";
int lastPacketMs = 0;
long packetCount  = 0;

int angleNow = ANGLE_MIN;
int distanceNow = 0;
int anglePrev = ANGLE_MIN;
int sweepDir = 1;

int[] echoCm = new int[181];
int[] echoStamp = new int[181];

PFont fontBig, fontSmall;
float cx, cy, R;

void setup()
{
  size(1200, 720);
  surface.setResizable(true);
  surface.setTitle("Arduino Ultrasonic Radar");
  smooth(8);

  fontBig = createFont(FONT_NAME, 26);
  fontSmall = createFont(FONT_NAME, 15);

  openPort();
}

void openPort()
{
  linkUp = false;
  if (port != null) { try { port.stop(); } catch (Exception e) { } port = null; }

  String chosen = choosePort();
  if (chosen == null) {
    println("!! No serial port found. Plug in the Arduino, then press R.");
    portLabel = "no port";
    return;
  }

  try{
    port = new Serial(this, chosen, BAUD);
    port.bufferUntil('\n');
    port.clear();
    linkUp    = true;
    portLabel = chosen;
    println("Connected to " + chosen + " at " + BAUD + " baud.");
  }
  catch (Exception e){
    println("!! Could not open " + chosen + ": " + e.getMessage());
    portLabel = "busy: " + chosen;
  }
}

String choosePort()
{
  String[] ports = Serial.list();
  println("--- serial ports visible to Processing ---");
  printArray(ports);
  println("------------------------------------------");

  if (FORCE_PORT.length() > 0) return FORCE_PORT;
  if (ports.length == 0) return null;

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

void serialEvent(Serial p)
{
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line);
  if (line.length() == 0) return;
  if (line.charAt(0) == '#') return;

  int comma = line.indexOf(',');
  if (comma < 1) return;

  int a, d;
  try {
    a = Integer.parseInt(line.substring(0, comma).trim());
    d = Integer.parseInt(line.substring(comma + 1).trim());
  }
  catch (NumberFormatException e) {
    return;
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
  else       { echoCm[a] = 0; }
}

void draw()
{
  background(3, 10, 6);

  cx = width / 2.0;
  cy = height * 0.90;
  R = min(width * 0.46, height * 0.80);

  drawGrid();
  drawContacts();
  drawSweep();
  drawHud();
}

float sx(float deg, float r) {
  float x = r * cos(radians(deg));
  return cx + (MIRROR_X ? -x : x);
}
float sy(float deg, float r) {
  return cy - r * sin(radians(deg));
}

void drawGrid()
{
  noFill();
  stroke(0, 190, 95, 110);
  strokeWeight(1.5);

  for (int i = 1; i <= 4; i++) {
    float dia = 2 * R * i / 4.0;
    arc(cx, cy, dia, dia, PI, TWO_PI);
  }

  for (int a = 0; a <= 180; a += 30) line(cx, cy, sx(a, R), sy(a, R));
  line(cx - R, cy, cx + R, cy);

  noStroke();
  fill(255, 255, 255, 8);
  arc(cx, cy, 2 * R, 2 * R, PI, PI + radians(ANGLE_MIN));
  arc(cx, cy, 2 * R, 2 * R, TWO_PI - radians(180 - ANGLE_MAX), TWO_PI);

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

void drawContacts()
{
  int now = millis();

  for (int a = 0; a <= 180; a++) {
    int d = echoCm[a];
    if (d <= 0) continue;

    int age = now - echoStamp[a];
    if (age > TRAIL_MS) continue;

    float life = 1.0 - (float) age / TRAIL_MS;
    float pr   = map(constrain(d, 0, MAX_RANGE_CM), 0, MAX_RANGE_CM, 0, R);
    float px   = sx(a, pr), py = sy(a, pr);

    stroke(255, 70, 70, 85 * life);
    strokeWeight(2);
    line(px, py, sx(a, R), sy(a, R));

    noStroke();
    fill(255, 90, 90, 60 * life);
    ellipse(px, py, 20, 20);
    fill(255, 130, 130, 255 * life);
    ellipse(px, py, 8, 8);
  }
}

void drawSweep()
{
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

void keyPressed()
{
  if (key == 's' || key == 'S') {
    saveFrame("screenshots/radar-####.png");
    println("screenshot saved into the sketch folder");
  }
  if (key == 'c' || key == 'C') {
    for (int i = 0; i <= 180; i++) { echoCm[i] = 0; echoStamp[i] = 0; }
  }
  if (key == 'r' || key == 'R') openPort();
}
