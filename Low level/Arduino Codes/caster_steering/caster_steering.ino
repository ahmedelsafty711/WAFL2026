// ========================
// USER SETTINGS
// ========================
const float PPR = 23700.0;  
const int pinA = 2;
const int pinB = 3;
const int motorPWM = 6;
const int motorDIR = 7;

volatile long encoderCount = 0;

// ========================
// ENCODER ISR
// ========================
void encoderA_ISR() {
  int b = digitalRead(pinB);
  if (b == HIGH) encoderCount++;
  else encoderCount--;
}

// ========================
void setup() {
  Serial.begin(115200);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), encoderA_ISR, RISING);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  Serial.println("Enter target angle:");
}

// ========================
void motorForward(int s) {
  digitalWrite(motorDIR, HIGH);
  analogWrite(motorPWM, s);
}

void motorBackward(int s) {
  digitalWrite(motorDIR, LOW);
  analogWrite(motorPWM, s);
}

void motorStop() {
  analogWrite(motorPWM, 0);
}

// ========================
// MAIN
// ========================
void loop() {

  static bool moving = false;
  static long targetPulse = 0;
  static bool directionSet = false;

  // --------- SERIAL INPUT ---------
  // --------- SERIAL INPUT ---------
if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();    // remove spaces and invisible chars

    if (input.length() == 0) return;  // ignore empty lines

    if (!input.equals("0") && input.toFloat() == 0) {
        // Means parseFloat returned 0 but user didn’t actually send "0"
        Serial.println("Ignored accidental zero (noise).");
        return;
    }

    float angle = input.toFloat();
    targetPulse = (long)(angle / 360.0 * PPR);

    Serial.print("User angle = ");
    Serial.println(angle);
    Serial.print("Target pulses = ");
    Serial.println(targetPulse);

    moving = true;
    directionSet = false;
}


  // --------- PRINT CURRENT POSITION ---------
  long pulses = encoderCount;
  float currentAngle = (pulses * 360.0) / PPR;

  Serial.print("Pulses: ");
  Serial.print(pulses);
  Serial.print("   Angle: ");
  Serial.println(currentAngle);

  delay(50);  // Slow print rate

  // --------- MOVEMENT CONTROL ---------
  if (moving) {

    long error = targetPulse - encoderCount;

    // ---- SET DIRECTION ONCE ----
    if (!directionSet) {
      if (error > 0) {
        motorForward(180);
        Serial.println("Moving CW...");
      } else {
        motorBackward(180);
        Serial.println("Moving CCW...");
      }
      directionSet = true;
    }

    // ---- STOP CONDITION ----
    if (abs(error) < 80) {
      motorStop();
      moving = false;

      Serial.println("==== Reached Target ====");
      Serial.print("Final Pulses: ");
      Serial.println(encoderCount);
      Serial.print("Final Angle: ");
      Serial.println((encoderCount * 360.0) / PPR);
      Serial.println("========================");

      return;
    }

    // ---- BOUNCE-PROOF OVERSHOOT CHECK ----
    if (directionSet) {
      if (error < 0 && digitalRead(motorDIR) == HIGH) {
        motorStop();
        moving = false;
        Serial.println("Overshoot CW — Stopped (No Bounce).");
      }
      if (error > 0 && digitalRead(motorDIR) == LOW) {
        motorStop();
        moving = false;
        Serial.println("Overshoot CCW — Stopped (No Bounce).");
      }
    }
  }
}
