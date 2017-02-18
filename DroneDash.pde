import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;
Serial port;

final byte header_len = 3;
final byte x8r_offset = header_len;
final byte x8r_len = 32;
final byte bno_offset = x8r_offset + x8r_len;
final byte bno_len = 52;
final byte bmp_offset = bno_offset + bno_len;
final byte bmp_len = 0;
final int  motor_offset = bmp_offset + bmp_len;
final byte motor_len = 12;
final byte ws2812_offset = motor_offset + motor_len;
final int  ws2812_len = 0; // 252
final int  packet_len = header_len + x8r_len + bno_len + bmp_len + motor_len + ws2812_len;

char[] dataPacket = new char[packet_len];
short[] x8r = new short[x8r_len/2];
int[] bno = new int[bno_len/4];
int[] bmp = new int[0];
char[][] ws2812 = new char[84][3];
short[] motor = new short[motor_len / 2];

char status = 0x00;
char onboard_leds = 0x00;

int serialCount = 0;
int aligned = 0;
int interval = 0;
int count = 0;

final int screen_width = 800;
final int screen_height = 600;

color color0 = #000000;
color color1 = #ff9900;
color color2 = #ff0000;
color color3 = #888888;

void display_panel_border() {
  stroke(color1);
  strokeWeight(3);
  noFill();
  rect(10, 10, screen_width - 20, screen_height - 20);
}

void display_leds() {
  float sh = 28.0f;
  float sw = 16.0f;
  float sx = 100.0f;
  float sy = 62.5f;
  float so = 3.0f;
  
  //  ###  RED  ###  //
  if ((onboard_leds & 0x01) > 0)
    stroke(#ff0000);
  else
    stroke(#660000);
    
  noFill();
  beginShape();
  vertex(sx - (sw / 2.0f), sy - (sh / 2.0f) - so);
  vertex(sx, sy - sh - so);
  vertex(sx + (sw / 2.0f), sy - (sh / 2.0f) - so);
  vertex(sx, sy - so);
  endShape(CLOSE);
  
  //  ###  ORANGE  ###  //
  if ((onboard_leds & 0x02) > 0)
    stroke(#ff8800);
  else
    stroke(#502300);
    
  noFill();
  beginShape();
  vertex(sx - sh - so, sy);
  vertex(sx - (sh / 2.0f) - so, sy - (sw / 2.0f));
  vertex(sx - so, sy);
  vertex(sx - (sh / 2.0f) - so, sy + (sw / 2.0f));
  endShape(CLOSE);
  
  //  ###  GREEN  ###  //
  if ((onboard_leds & 0x04) > 0)
    stroke(#00ff00);
  else
    stroke(#004400);
    
  noFill();
  beginShape();
  vertex(sx - (sw / 2.0f), sy + (sh / 2.0f) + so);
  vertex(sx, sy + sh + so);
  vertex(sx + (sw / 2.0f), sy + (sh / 2.0f) + so);
  vertex(sx, sy + so);
  endShape(CLOSE);
  
  //  ###  BLUE  ###  //
  if ((onboard_leds & 0x08) > 0)
    stroke(#0000ff);
  else
    stroke(#000066);
    
  noFill();
  beginShape();
  vertex(sx + sh + so, sy);
  vertex(sx + (sh / 2.0f) + so, sy - (sw / 2.0f));
  vertex(sx + so, sy);
  vertex(sx + (sh / 2.0f) + so, sy + (sw / 2.0f));
  endShape(CLOSE);
}

void display_panel_heading() {
  stroke(color1);
  noFill();
  rect(20, 20, screen_width - 40, screen_height / 7);
  
  textSize(30);
  fill(color1);
  textAlign(CENTER, CENTER);
  text("Drone Dashboard", (screen_width / 2), (screen_height / 14) + 5);
  
  textSize(12);
  fill(color3);
  textAlign(CENTER, TOP);
  text("Status: ", (screen_width / 2) - 40, (screen_height / 14) + 40);
  
  String s = "Unknown Status!";
  
  if (status == 0)
    s = "Calibrating...";
   else if (status == 1)
     s = "Idle";
    else if (status == 2)
      s = "Arming...";
    else if (status == 3)
      s = "ARMED!";
  
  fill(color2);
  textAlign(LEFT, TOP);
  text(s, (screen_width / 2), (screen_height / 14) + 40);
  
  display_leds();
}

void display_x8r_pluses(int x1, int y1, int x2, int y2) {
  x1 = clamp(x1, 172, 1811);
  y1 = clamp(y1, 172, 1811);
  x2 = clamp(x2, 172, 1811);
  y2 = clamp(y2, 172, 1811);
  
  float pw = 8.0f;
  float pr = pw / 2.0f;
  float bw = 80.0f - pw;
  
  float c = (bw / (1811 - 172));
  float lx = (x1 - 992) * c;
  float ly = (y1 - 992) * c;
  float rx = (x2 - 992) * c;
  float ry = (y2 - 992) * c;
  
  float offset_lx = 110.0f - pr;
  float offset_ly = 210.0f;
  float offset_rx = 210.0f - pr;
  float offset_ry = offset_ly;
  
  float x[] = { lx+offset_lx, lx+(offset_lx+pw), lx+(offset_lx+pr), lx+(offset_lx+pr), 
                rx+offset_rx, rx+(offset_rx+pw), rx+(offset_rx+pr), rx+(offset_rx+pr) };
  float y[] = { -ly+offset_ly, -ly+offset_ly, (-ly+offset_ly)-pr, (-ly+offset_ly)+pr, 
                -ry+offset_ry, -ry+offset_ry, (-ry+offset_ry)-pr, (-ry+offset_ry)+pr };
  
  stroke(#ff0000);
  strokeWeight(2);
  for (int i = 0; i < 8; i+=2)
    line(x[i], y[i], x[i+1], y[i+1]);
}

void display_x8r_switches() {
  color c;
  strokeWeight(4);
  
  // LEFT SWITCHES
  for (int i = 0; i < 4; i++) {
    int k = 4 + i;
    if (i == 3)
      k = 10;
    if (i == 0)
      k = 5;
    if (i == 1)
      k = 4;
      
    if (x8r[k] < 200)
      c = color(#00dd00);
    else if (x8r[k] < 1000)
      c = color(#dddd00);
    else
      c = color(#dd0000);
      
    stroke(c);
    line(118 - (i * 18), 164, 98 - (i * 18), 144);
  }
  
  // RIGHT SWITCHES
  for (int i = 0; i < 4; i++) {
    int k = 7 + i;
    if (i == 3)
      k = 11;
      
    if (x8r[k] < 200)
      c = color(#00dd00);
    else if (x8r[k] < 1000)
      c = color(#dddd00);
    else
      c = color(#dd0000);
      
    stroke(c);
    line(203 + (i * 18), 164, 223 + (i * 18), 144);
  }
  
  // LEFT KNOB
  stroke(color1);
  strokeWeight(3);
  noFill();
  ellipse(135, 154, 20, 20);
  
  pushMatrix();
  translate(135, 154);
  rotate((315 * ((x8r[12] - 172.0f) / (1811 - 172))) * (PI / 180.0f) + (PI / 8));
  stroke(#ffdddd);
  strokeWeight(3);
  line(0, 3, 0, 12);
  popMatrix();
  
  // RIGHT KNOB
  stroke(color1);
  strokeWeight(3);
  noFill();
  ellipse(185, 154, 20, 20);
  
  pushMatrix();
  translate(185, 154);
  rotate((315 * ((x8r[13] - 172.0f) / (1811 - 172))) * (PI / 180.0f) + (PI / 8));
  stroke(#ffdddd);
  strokeWeight(3);
  line(0, 3, 0, 12);
  popMatrix();
  
  // LEFT SLIDER
  float v = 180.0f * (clamp(x8r[14], 172, 1811) - 172.0f) / (1811.0f - 172.0f);
  stroke(color1);
  strokeWeight(2);
  fill(#0000dd);
  arc(64, 190, 30, 30, (90) * PI / 180.0f, (90 + v) * PI / 180.0f, PIE);
  fill(color1);
  arc(64, 190, 30, 30, (90 + v) * PI / 180.0f, (270) * PI / 180.0f, PIE);
  
  
  // RIGHT SLIDER
  v = 180.0f * (clamp(x8r[15], 172, 1811) - 172.0f) / (1811.0f - 172.0f);
  arc(256, 190, 30, 30, (270) * PI / 180.0f, (270 + (180 - v)) * PI / 180.0f, PIE);
  fill(#0000dd);
  arc(256, 190, 30, 30, (270 + (180 - v)) * PI / 180.0f, (450) * PI / 180.0f, PIE);
  
  strokeWeight(2);
}

void display_gauge_x8r() {
  stroke(#ffffff);
  strokeWeight(2);
  noFill();
  rect(70, 170, 80, 80);
  rect(170, 170, 80, 80);
  
  display_x8r_pluses(x8r[3], x8r[0], x8r[1], x8r[2]);
  display_x8r_switches();
}

void display_gauge_attitude() {
  noFill();
  stroke(color1);
  strokeWeight(5);
  ellipse(400, 190, 126, 126);
  stroke(#ffdddd);
  strokeWeight(1.5);
  fill(#3377ff);
  float pitch = bno[0] / 100.0f;
  float roll = bno[1] / 100.0f;
  arc(400, 190, 122, 122, (180-pitch-roll) * PI / 180.0f, (360+pitch-roll) * PI / 180.0f, CHORD);
  fill(#773300);
  arc(400, 190, 122, 122, (0+pitch-roll) * PI / 180.0f, (180.0f-pitch-roll) * PI / 180.0f, CHORD);
  
  stroke(color1);
  strokeWeight(2);
  line(400, 150, 400, 230);
  strokeWeight(1);
  line(393, 160, 407, 160);
  line(390, 170, 410, 170);
  line(393, 180, 407, 180);
  line(390, 210, 410, 210);
  line(393, 220, 407, 220);
  strokeWeight(2);
  line(350, 190, 380, 190);
  line(420, 190, 445, 190);
  strokeWeight(1);
  stroke(color0);
  line(380, 190, 390, 200);
  line(390, 200, 410, 200);
  line(410, 200, 420, 190);
  
  stroke(color1);
  strokeWeight(2);
}

void display_gauge_compass() {
  PImage img;
  img = loadImage("compass.png");
  
  noFill();
  stroke(color1);
  strokeWeight(5);
  ellipse(550, 190, 126, 126);
  image(img, 506, 146, 90, 90);
  
  textSize(16);
  fill(color1);
  textAlign(CENTER, TOP);
  text("N", 550, 129);
  text("S", 551, 232);
  text("E", 601, 180);
  text("W", 500, 180);
  
  pushMatrix();
  translate(550, 191);
  fill(#ff0000);
  stroke(color0);
  strokeWeight(1);
  rotate(((bno[2] / 100.0f) * PI) / 180.0f);
  beginShape();
  vertex(-8, 8, 2);
  vertex(0, 0, 2);
  vertex(8, 8, 2);
  vertex(0, -45, 2);
  endShape(CLOSE);
  
  stroke(color0);
  fill(#eedddd);
  beginShape();
  vertex(-8, 8, 2);
  vertex(0, 0, 2);
  vertex(8, 8, 2);
  vertex(0, 45, 2);
  endShape(CLOSE);
  
  popMatrix();
  
  strokeWeight(2);
}

void display_gauge_pressure() {
  noFill();
  stroke(color1);
  strokeWeight(10);
  ellipse(700, 190, 120, 120);
  strokeWeight(3);
  arc(700, 190, 97, 97, 0.83 * PI, 2.17 * PI, OPEN);
  
  pushMatrix();
  
  translate(700, 190);
  rotate(PI / 3);
  for (short i = 0; i < 9; i++) {
    line(0, 48.5, 0, 40.5);
    rotate(PI / 6);
  }
  
  popMatrix();
  
  strokeWeight(3);
  fill(color0);
  ellipse(700, 190, 15, 15);
  
  textSize(14);
  fill(color1);
  textAlign(CENTER, TOP);
  text("hPa", 700, 215);
  
  pushMatrix();
  translate(700, 190);
  fill(#ff0000);
  stroke(#ff0000);
  strokeWeight(1);
  rotate(-5.35*PI/8);
  //rotate(10.5*PI/8);
  beginShape();
  vertex(-5, -6, -1);
  vertex(5, -6, -1);
  vertex(0, -37, -1);
  endShape(CLOSE);
  popMatrix();
  
  strokeWeight(2);
}

void display_panel_gauge() {
  stroke(color1);
  noFill();
  rect(20, 30 + (screen_height / 7), screen_width - 40, screen_height / 4);
  
  display_gauge_x8r();
  display_gauge_attitude();
  display_gauge_compass();
  display_gauge_pressure();
}

void display_panel_attitude() {
  stroke(color1);
  noFill();
  rect(20, 40 + (screen_height / 7) + (screen_height / 4), (screen_width - 40) / 2.5, (screen_width - 40) / 2.5);
}

void display_panel_value() {
  textSize(12);
  fill(color2);
  textAlign(CENTER, TOP);
  text("Gyroscope", (screen_width / 2), (screen_height / 2) + -15);
  text("Accelerometer", (screen_width / 2), (screen_height / 2) + 20);
  text("Magnetometer", (screen_width / 2), (screen_height / 2) + 55);
  text("Attitude", (screen_width / 2), (screen_height / 2) + 90);
  text("Temperature", (screen_width / 2), (screen_height / 2) + 125);
  text("Pressure", (screen_width / 2), (screen_height / 2) + 160);
  //text("Rate (Hz) (min/cur/max)", (screen_width / 2), (screen_height / 2) + 195);
  
  String str_gyro = String.format("%3.2f, %3.2f, %3.2f", (bno[3] / 100.0f), (bno[4] / 100.0f), (bno[5] / 100.0f));
  String str_accel = String.format("%3.2f, %3.2f, %3.2f", (bno[6] / 100.0f), (bno[7] / 100.0f), (bno[8] / 100.0f));
  String str_mag = String.format("%3.2f, %3.2f, %3.2f", (bno[9] / 100.0f), (bno[10] / 100.0f), (bno[11] / 100.0f));
  String str_att = String.format("%3.2f, %3.2f, %3.2f", (bno[0] / 100.0f), (bno[1] / 100.0f), (bno[2] / 100.0f));
  String str_temp = String.format("%3.1f *F", (bno[12] / 100.0f));
  //String str_rate_mtr = String.format("MTR: %d, %d, %d", rate_min[6], rate[6], rate_max[6]);
  //String str_rate_bno = String.format("BNO: %d, %d, %d", rate_min[1], rate[1], rate_max[1]);
  //String str_rate_x8r = String.format("X8R: %d, %d, %d", rate_min[0], rate[0], rate_max[0]);
  //String str_rate_led = String.format("LED: %d, %d, %d", rate_min[7], rate[7], rate_max[7]);
  
  textSize(10);
  fill(color3);
  textAlign(CENTER, TOP);
  text(str_gyro, (screen_width / 2), (screen_height / 2) + 0);
  text(str_accel, (screen_width / 2), (screen_height / 2) + 35);
  text(str_mag, (screen_width / 2), (screen_height / 2) + 70);
  text(str_att, (screen_width / 2), (screen_height / 2) + 105);
  text(str_temp, (screen_width / 2), (screen_height / 2) + 140);
  text("N/A", (screen_width / 2), (screen_height / 2) + 175);
  //text(str_rate_mtr, (screen_width / 2), (screen_height / 2) + 210);
  //text(str_rate_bno, (screen_width / 2), (screen_height / 2) + 225);
  //text(str_rate_x8r, (screen_width / 2), (screen_height / 2) + 240);
  //text(str_rate_led, (screen_width / 2), (screen_height / 2) + 255);
}

void display_panel_motor() {
  stroke(color1);
  noFill();
  rect(screen_width - 20 - ((screen_width - 40) / 2.5), 40 + (screen_height / 7) + (screen_height / 4), (screen_width - 40) / 2.5, (screen_width - 40) / 2.5);
  
  float arm_length = 75.0f;
  float motor_offset = 45.0f;
  float center[] = { screen_width - 172.5f, screen_height - 172.5f };
  float led_size = 4.5f;
  
  pushMatrix();
  
  translate(center[0], center[1]);  // -300, -45
  
  for (int i = 0; i < 6; i++) {
    noFill();
    stroke(color1);
    strokeWeight(2);
    rotate(-PI/4.0f);
    if (i == 1 || i == 4)
      rotate(-PI/4.0f);
    line(25, -5, arm_length + 25, -5);
    line(25, 5, arm_length + 25, 5);
    strokeWeight(3);
    ellipse(arm_length + motor_offset, 0, 40, 40);
    strokeWeight(2);
    
    color cm = #000099;
    float val = (motor[i] - 1068.0f) / 800.0f;
    if ((motor[i] - 1068) >= 0) {
      if (val < 0.5f)
        cm = color(255 * (val * 2.0f), 255, 0);
      else
        cm = color(255, 255 * (1.0f - ((val - 0.5f) * 2.0f)), 0);
    }
    
    fill(cm);
    ellipse(arm_length + motor_offset, 0, 28, 28);
    for (int j = 0; j < 11; j++) {
      stroke(ws2812[(i * 11) + (i * 3) + j][0], ws2812[(i * 11) + (i * 3) + j][1], ws2812[(i * 11) + (i * 3) + j][2]);
      fill(ws2812[(i * 11) + (i * 3) + j][0], ws2812[(i * 11) + (i * 3) + j][1], ws2812[(i * 11) + (i * 3) + j][2]);
      ellipse(28.2 + (6.7*j), 0, led_size, led_size);
    }
    stroke(ws2812[(i * 11) + (i * 3) + 11][0], ws2812[(i * 11) + (i * 3) + 11][1], ws2812[(i * 11) + (i * 3) + 11][2]);
    fill(ws2812[(i * 11) + (i * 3) + 11][0], ws2812[(i * 11) + (i * 3) + 11][1], ws2812[(i * 11) + (i * 3) + 11][2]);
    ellipse(142, 16, led_size, led_size);
    stroke(ws2812[(i * 11) + (i * 3) + 12][0], ws2812[(i * 11) + (i * 3) + 12][1], ws2812[(i * 11) + (i * 3) + 12][2]);
    fill(ws2812[(i * 11) + (i * 3) + 12][0], ws2812[(i * 11) + (i * 3) + 12][1], ws2812[(i * 11) + (i * 3) + 12][2]);
    ellipse(147, 0, led_size, led_size);
    stroke(ws2812[(i * 11) + (i * 3) + 13][0], ws2812[(i * 11) + (i * 3) + 13][1], ws2812[(i * 11) + (i * 3) + 13][2]);
    fill(ws2812[(i * 11) + (i * 3) + 13][0], ws2812[(i * 11) + (i * 3) + 13][1], ws2812[(i * 11) + (i * 3) + 13][2]);
    ellipse(142, -16, led_size, led_size);
  }
  
  noFill();
  stroke(color1);
  rotate(PI/8.0f);
  strokeWeight(4);
  polygon(0, 0, 25, 8);
  fill(color1);
  strokeWeight(1);
  polygon(0, 0, 17, 8);
  
  popMatrix();
}

void setup() {
  size(800, 600, OPENGL);
  gfx = new ToxiclibsSupport(this);
  
  lights();
  smooth();
  
  port = new Serial(this, "/dev/cu.SLAB_USBtoUART", 230400);
}

void draw() {
  background(color0);
  
  pushMatrix();
  //rect(20, 40 + (screen_height / 7) + (screen_height / 4), (screen_width - 40) / 2.5, (screen_width - 40) / 2.5);
  
  translate(183, 4.9 * (height / 7));
  
  float ypr[] = new float[3];
  ypr[0] = ((bno[2] / 100.0f) * PI) / 180.0f;  // YAW
  ypr[1] = ((bno[1] / 100.0f) * PI) / 180.0f;  // ROLL
  ypr[2] = ((bno[0] / 100.0f) * PI) / 180.0f;  // PITCH
  
  rotateY(-ypr[0]);
  rotateZ(ypr[1]);
  rotateX(-ypr[2]);
  ////float[] axis = quat.toAxisAngle();
  ////rotate(axis[0], -axis[1], axis[3], axis[2]);

  stroke(color0);
  strokeWeight(1);
  // draw main body in red
  fill(255, 0, 0, 200);
  box(10, 10, 200);
  
  // draw front-facing tip in blue
  fill(0, 0, 255, 200);
  pushMatrix();
  translate(0, 0, -120);
  rotateX(PI/2);
  drawCylinder(0, 20, 20, 8);
  popMatrix();
  
  // draw wings and tail fin in green
  fill(0, 255, 0, 200);
  beginShape(TRIANGLES);
  vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  // wing top layer
  vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  // wing bottom layer
  vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  // tail left layer
  vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  // tail right layer
  endShape();
  beginShape(QUADS);
  vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
  vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
  vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
  vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
  vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
  vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
  endShape();
  
  popMatrix();
  
  display_panel_border();
  display_panel_heading();
  display_panel_gauge();
  display_panel_attitude();
  display_panel_value();
  display_panel_motor();
}

void serialEvent(Serial port) {
    interval = millis();
   
    while (port.available() > 0) {
        int ch = port.read();
        if (aligned < 3) {
            if (ch == 0xFE) serialCount = 0;
            if (serialCount == 0) {
                if (ch == 0xFE) aligned++; else aligned = 0;
            } else if (serialCount == 1) {
                if (ch == 0xDD) aligned++; else aligned = 0;
            } else if (serialCount == packet_len - 1)
                if (ch == '\n') aligned++; else aligned = 0;
            
            serialCount++;
            if (serialCount == packet_len) serialCount = 0;
        } else {
            if (serialCount > 0 || ch == 0xFE) {
                dataPacket[serialCount++] = (char)ch;
                if (serialCount == packet_len) {
                    serialCount = 0;
                    
                    status = (char)(dataPacket[2] >> 4);
                    onboard_leds = (char)(dataPacket[2] & 0x0F);
                    
                    for (int i = x8r_offset; i < x8r_offset + x8r_len; i += 2)
                      x8r[(i - x8r_offset) / 2] = (short)((dataPacket[i] << 8) | dataPacket[i + 1]);
                      
                    for (int i = bno_offset; i < bno_offset + bno_len; i += 4)
                      bno[(i - bno_offset) / 4] = (int)((dataPacket[i] << 24) | (dataPacket[i+1] << 16) | (dataPacket[i+2] << 8) | dataPacket[i+3]);
                    
                    for (int i = bmp_offset; i < bmp_offset + bmp_len; i += 2)
                      bmp[(i - bmp_offset) / 2] = (short)((dataPacket[i] << 8) | dataPacket[i + 1]);
                    
                    for (int i = motor_offset; i < motor_offset + motor_len; i += 2)
                      motor[(i - motor_offset) / 2] = (short)((dataPacket[i] << 8) | dataPacket[i + 1]);
                    
                    //for (int i = ws2812_offset; i < ws2812_offset + ws2812_len; i++)
                    //  ws2812[(i - ws2812_offset) / 3][(i - ws2812_offset) % 3] = dataPacket[i];
                    
                    /*
                    q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
                    q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
                    q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
                    q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
                    for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
                    
                    quat.set(q[0], q[1], q[2], q[3]);
                    */
                }
            }
        }
    }
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}

void polygon(float x, float y, float radius, int npoints) {
  float angle = TWO_PI / npoints;
  beginShape();
  for (float a = 0; a < TWO_PI; a += angle) {
    float sx = x + cos(a) * radius;
    float sy = y + sin(a) * radius;
    vertex(sx, sy);
  }
  endShape(CLOSE);
}

int clamp(int value, int min, int max) {
  if (value < min)
    return min;
   if (value > max)
     return max;
    
    return value;
}

float clamp_f(float value, float min, float max) {
  if (value < min)
    return min;
   if (value > max)
     return max;
    
    return value;
}