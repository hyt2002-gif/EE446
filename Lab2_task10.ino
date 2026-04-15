#include <PDM.h>                   // microphone
#include <Arduino_APDS9960.h>      // light and proximity
#include <Arduino_BMI270_BMM150.h> // IMU

short microphone_sample[256];
volatile int samples = 0;
float microphone_level = 0.0;

int bright_value = 0;
int proximity_value = 0;
float moving_value = 0.0;

float prev_x = 0.0;
float prev_y = 0.0;
float prev_z = 0.0;
bool have_prev_acce = false;

const float SOUND_THRESHOLD = 60.0;
const int BRIGHT_THRESHOLD = 100;
const float MOVING_THRESHOLD = 0.10;
const int PROXIMITY_THRESHOLD = 120;

const unsigned long PRINT_INTERVAL_MS = 5000;
unsigned long lastPrint = 0;

// --------------------------------------------------
// Microphone callback
// --------------------------------------------------
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(microphone_sample, bytesAvailable);
  samples = bytesAvailable / 2;   // 2 bytes per short sample
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // Start APDS9960 (light + proximity)
  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  // Start IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  // Start microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone.");
    while (1);
  }

  Serial.println("Workspace awareness system started");
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------
void loop() {
  updateMicrophone();
  updateLightAndProximity();
  updateMotion();

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    classifyAndPrint();
  }
}

// --------------------------------------------------
// Update microphone activity level
// --------------------------------------------------
void updateMicrophone() {
  if (samples > 0) {
    noInterrupts();

    int n = samples;
    if (n > 256) {
      n = 256;
    }

    long sum = 0;
    for (int i = 0; i < n; i++) {
      sum += abs(microphone_sample[i]);
    }

    samples = 0;
    interrupts();

    float current_level = 0.0;
    if (n > 0) {
      current_level = (float)sum / n;
    }

    // smoothing
    microphone_level = 0.7 * microphone_level + 0.3 * current_level;
  }
}

// --------------------------------------------------
// Update light and proximity
// bright_value stores the clear channel value
// --------------------------------------------------
void updateLightAndProximity() {
  if (APDS.colorAvailable()) {
    int r, g, b, c;
    APDS.readColor(r, g, b, c);
    bright_value = c;
  }

  if (APDS.proximityAvailable()) {
    proximity_value = APDS.readProximity();
  }
}

// --------------------------------------------------
// Update motion from accelerometer change
// --------------------------------------------------
void updateMotion() {
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);

    if (!have_prev_acce) {
      prev_x = x;
      prev_y = y;
      prev_z = z;
      have_prev_acce = true;
      moving_value = 0.0;
      return;
    }

    float dx = x - prev_x;
    float dy = y - prev_y;
    float dz = z - prev_z;

    float current_motion = sqrt(dx * dx + dy * dy + dz * dz);

    // smoothing
    moving_value = 0.7 * moving_value + 0.3 * current_motion;

    prev_x = x;
    prev_y = y;
    prev_z = z;
  }
}

// --------------------------------------------------
// Classify and print
// --------------------------------------------------
void classifyAndPrint() {
  int sound = (microphone_level > SOUND_THRESHOLD) ? 1 : 0;
  int dark = (bright_value < BRIGHT_THRESHOLD) ? 1 : 0;
  int moving = (moving_value > MOVING_THRESHOLD) ? 1 : 0;
  int near = (proximity_value > PROXIMITY_THRESHOLD) ? 1 : 0;

  const char* final_label;

  // exact preferred combinations first
  if (sound == 0 && dark == 0 && moving == 0 && near == 0) {
    final_label = "QUIET_BRIGHT_STEADY_FAR";
  } else if (sound == 1 && dark == 0 && moving == 0 && near == 0) {
    final_label = "NOISY_BRIGHT_STEADY_FAR";
  } else if (sound == 0 && dark == 1 && moving == 0 && near == 1) {
    final_label = "QUIET_DARK_STEADY_NEAR";
  } else if (sound == 1 && dark == 0 && moving == 1 && near == 1) {
    final_label = "NOISY_BRIGHT_MOVING_NEAR";
  }
  // fallback rules so every case still maps to one of the 4 labels
  else if (moving && near) {
    final_label = "NOISY_BRIGHT_MOVING_NEAR";
  } else if (dark && near) {
    final_label = "QUIET_DARK_STEADY_NEAR";
  } else if (sound) {
    final_label = "NOISY_BRIGHT_STEADY_FAR";
  } else {
    final_label = "QUIET_BRIGHT_STEADY_FAR";
  }

  // line 1: raw values
  Serial.print("raw,mic=");
  Serial.print((int)microphone_level);
  Serial.print(",clear=");
  Serial.print(bright_value);
  Serial.print(",motion=");
  Serial.print(moving_value, 3);
  Serial.print(",prox=");
  Serial.println(proximity_value);

  // line 2: binary flags
  Serial.print("flags,sound=");
  Serial.print(sound);
  Serial.print(",dark=");
  Serial.print(dark);
  Serial.print(",moving=");
  Serial.print(moving);
  Serial.print(",near=");
  Serial.println(near);

  // line 3: final state
  Serial.print("state,");
  Serial.println(final_label);
}
