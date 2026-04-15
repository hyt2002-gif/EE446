#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>

const float HUMID_JUMP_THRESHOLD = 4.0;
const float TEMP_RISE_THRESHOLD = 0.6;
const float MAG_SHIFT_THRESHOLD = 12.0;
const int CLEAR_CHANGE_THRESHOLD = 100;
const int RGB_CHANGE_THRESHOLD = 45;

const unsigned long LOOP_DELAY_MS = 200;
const unsigned long EVENT_COOLDOWN_MS = 1200;

const int BASELINE_SAMPLES = 20;
const float BASELINE_ALPHA = 0.02;

float baseRH = 0.0;
float baseTemp = 0.0;

float baseMx = 0.0;
float baseMy = 0.0;
float baseMz = 0.0;

float baseR = 0.0;
float baseG = 0.0;
float baseB = 0.0;
float baseClear = 0.0;

// Last valid readings
float lastMx = 0.0;
float lastMy = 0.0;
float lastMz = 0.0;

int lastR = 0;
int lastG = 0;
int lastB = 0;
int lastClear = 0;

// Cooldown
String lastEvent = "BASELINE_NORMAL";
unsigned long lastEventTime = 0;

bool readMagData(float &mx, float &my, float &mz) {
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    return true;
  }
  return false;
}

bool readColorData(int &r, int &g, int &b, int &c) {
  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, c);
    return true;
  }
  return false;
}

float magneticMetric(float mx, float my, float mz) {
  float dx = mx - baseMx;
  float dy = my - baseMy;
  float dz = mz - baseMz;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

void calibrateBaseline() {
  float sumRH = 0.0;
  float sumTemp = 0.0;
  float sumMx = 0.0;
  float sumMy = 0.0;
  float sumMz = 0.0;
  float sumR = 0.0;
  float sumG = 0.0;
  float sumB = 0.0;
  float sumClear = 0.0;

  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    float rh = HS300x.readHumidity();
    float temp = HS300x.readTemperature();

    float mx, my, mz;
    while (!readMagData(mx, my, mz)) {
      delay(5);
    }

    int r, g, b, c;
    while (!readColorData(r, g, b, c)) {
      delay(5);
    }

    sumRH += rh;
    sumTemp += temp;
    sumMx += mx;
    sumMy += my;
    sumMz += mz;
    sumR += r;
    sumG += g;
    sumB += b;
    sumClear += c;

    delay(50);
  }

  baseRH = sumRH / BASELINE_SAMPLES;
  baseTemp = sumTemp / BASELINE_SAMPLES;

  baseMx = sumMx / BASELINE_SAMPLES;
  baseMy = sumMy / BASELINE_SAMPLES;
  baseMz = sumMz / BASELINE_SAMPLES;

  baseR = sumR / BASELINE_SAMPLES;
  baseG = sumG / BASELINE_SAMPLES;
  baseB = sumB / BASELINE_SAMPLES;
  baseClear = sumClear / BASELINE_SAMPLES;

  lastMx = baseMx;
  lastMy = baseMy;
  lastMz = baseMz;

  lastR = (int)baseR;
  lastG = (int)baseG;
  lastB = (int)baseB;
  lastClear = (int)baseClear;
}

void updateBaselineNormal(float rh, float temp, float mx, float my, float mz, int r, int g, int b, int c) {
  baseRH = (1.0 - BASELINE_ALPHA) * baseRH + BASELINE_ALPHA * rh;
  baseTemp = (1.0 - BASELINE_ALPHA) * baseTemp + BASELINE_ALPHA * temp;

  baseMx = (1.0 - BASELINE_ALPHA) * baseMx + BASELINE_ALPHA * mx;
  baseMy = (1.0 - BASELINE_ALPHA) * baseMy + BASELINE_ALPHA * my;
  baseMz = (1.0 - BASELINE_ALPHA) * baseMz + BASELINE_ALPHA * mz;

  baseR = (1.0 - BASELINE_ALPHA) * baseR + BASELINE_ALPHA * r;
  baseG = (1.0 - BASELINE_ALPHA) * baseG + BASELINE_ALPHA * g;
  baseB = (1.0 - BASELINE_ALPHA) * baseB + BASELINE_ALPHA * b;
  baseClear = (1.0 - BASELINE_ALPHA) * baseClear + BASELINE_ALPHA * c;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS3003 sensor.");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  calibrateBaseline();
}

void loop() {
  float rh = HS300x.readHumidity();
  float temp = HS300x.readTemperature();

  float mx, my, mz;
  if (readMagData(mx, my, mz)) {
    lastMx = mx;
    lastMy = my;
    lastMz = mz;
  } else {
    mx = lastMx;
    my = lastMy;
    mz = lastMz;
  }

  int r, g, b, c;
  if (readColorData(r, g, b, c)) {
    lastR = r;
    lastG = g;
    lastB = b;
    lastClear = c;
  } else {
    r = lastR;
    g = lastG;
    b = lastB;
    c = lastClear;
  }

  float mag = magneticMetric(mx, my, mz);

  int humid_jump = ((rh - baseRH) >= HUMID_JUMP_THRESHOLD) ? 1 : 0;
  int temp_rise = ((temp - baseTemp) >= TEMP_RISE_THRESHOLD) ? 1 : 0;
  int mag_shift = (mag >= MAG_SHIFT_THRESHOLD) ? 1 : 0;

  int clear_change = (abs(c - (int)round(baseClear)) >= CLEAR_CHANGE_THRESHOLD) ? 1 : 0;
  int rgb_change =
      (abs(r - (int)round(baseR)) >= RGB_CHANGE_THRESHOLD ||
       abs(g - (int)round(baseG)) >= RGB_CHANGE_THRESHOLD ||
       abs(b - (int)round(baseB)) >= RGB_CHANGE_THRESHOLD) ? 1 : 0;

  int light_or_color_change = (clear_change || rgb_change) ? 1 : 0;

  String currentEvent = "BASELINE_NORMAL";

  if (mag_shift) {
    currentEvent = "MAGNETIC_DISTURBANCE_EVENT";
  } else if (humid_jump || temp_rise) {
    currentEvent = "BREATH_OR_WARM_AIR_EVENT";
  } else if (light_or_color_change) {
    currentEvent = "LIGHT_OR_COLOR_CHANGE_EVENT";
  } else {
    currentEvent = "BASELINE_NORMAL";
  }

  // cooldown to avoid rapid repeated retriggering
  if (currentEvent != lastEvent) {
    if (millis() - lastEventTime >= EVENT_COOLDOWN_MS) {
      lastEvent = currentEvent;
      lastEventTime = millis();
    }
  }

  // slowly adapt baseline only during normal condition
  if (lastEvent == "BASELINE_NORMAL") {
    updateBaselineNormal(rh, temp, mx, my, mz, r, g, b, c);
  }

  // Required Serial Monitor format
  Serial.print("raw,rh=");
  Serial.print(rh, 2);
  Serial.print(",temp=");
  Serial.print(temp, 2);
  Serial.print(",mag=");
  Serial.print(mag, 2);
  Serial.print(",r=");
  Serial.print(r);
  Serial.print(",g=");
  Serial.print(g);
  Serial.print(",b=");
  Serial.print(b);
  Serial.print(",clear=");
  Serial.println(c);

  Serial.print("flags,humid_jump=");
  Serial.print(humid_jump);
  Serial.print(",temp_rise=");
  Serial.print(temp_rise);
  Serial.print(",mag_shift=");
  Serial.print(mag_shift);
  Serial.print(",light_or_color_change=");
  Serial.println(light_or_color_change);

  Serial.print("event,");
  Serial.println(lastEvent);

  delay(LOOP_DELAY_MS);
}
