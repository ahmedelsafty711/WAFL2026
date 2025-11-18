#define sig1 9
#define sig2 10
#define led 13
#define sw0 4
#define swUp 6
#define swDwn 8

void setup() {
  // put your setup code here, to run once:
  pinMode(sig1, OUTPUT);
  pinMode(sig2, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(sw0, INPUT_PULLUP);
  pinMode(swUp, INPUT_PULLUP);
  pinMode(swDwn, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(sw0) == 0) {
  digitalWrite(sig1, LOW);
  digitalWrite(sig2, LOW);
  Serial.println("Stop");
  }
  else if (digitalRead(swUp) == 0 && digitalRead(swDwn) == 1) {  // MOVES UP
  digitalWrite(sig1, HIGH);
  digitalWrite(sig2, LOW);
  Serial.println("Up");

  }
  else if (digitalRead(swDwn) == 0 && digitalRead(swUp) == 1) {  // Moves DOWN
  digitalWrite(sig1, LOW);
  digitalWrite(sig2, HIGH);
  Serial.println("Down");

  }
  else {
  digitalWrite(sig1, LOW);
  digitalWrite(sig2, LOW);
  Serial.println("Stop");

  }
}
