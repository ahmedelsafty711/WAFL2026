//*******************************************************************************//
//*     TITLE: Motor Control System with Limit Switches                         *//
//*     AUTHORS: Mohamed Montasser                                              *//
//*     DATE: 18/DEC/2025                                                       *//
//*     DESCRIPTION:                                                             *
//*     This program controls a motor using two relays for direction control     *
//*     with safety limit switches and manual control buttons.                   *
//*     Relay Wiring:                                                            *
//*     Relay1 - NC:GND - NO:VCC - Com:Yellow (Motor control line 1)             *
//*     Relay2 - NC:GND - NO:VCC - Com:White (Motor control line 2)              *
//*******************************************************************************//



// Pin Definitions
#define sig1 9        // Relay 1 control signal - controls motor direction
#define sig2 10       // Relay 2 control signal - controls motor direction  
#define led 13        // Status LED (usually built-in on Arduino)
#define sw0 4         // Emergency stop button (active LOW)
#define swUp 6        // Move up button (active LOW)
#define swDwn 8       // Move down button (active LOW)
#define LimitUp 2     // Upper limit switch - stops upward movement when triggered
#define LimitDown 3   // Lower limit switch - stops downward movement when triggered

void setup() {
  // Initialize digital pins
  pinMode(sig1, OUTPUT);      // Relay 1 control - set as output
  pinMode(sig2, OUTPUT);      // Relay 2 control - set as output
  pinMode(led, OUTPUT);       // Status LED - set as output
  
  // Initialize input pins with internal pull-up resistors
  // Inputs will read HIGH when not pressed, LOW when pressed/activated
  pinMode(sw0, INPUT_PULLUP);       // Emergency stop button
  pinMode(swUp, INPUT_PULLUP);      // Up movement button
  pinMode(swDwn, INPUT_PULLUP);     // Down movement button
  pinMode(LimitUp, INPUT_PULLUP);   // Upper limit switch
  pinMode(LimitDown, INPUT_PULLUP); // Lower limit switch
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Motor Control System Started");
}

void loop() {
  // Main control logic - checks conditions in priority order
  
  // Priority 1: Emergency Stop - if stop button is pressed
  if (digitalRead(sw0) == 0) {
    digitalWrite(sig1, LOW);   // Turn off relay 1
    digitalWrite(sig2, LOW);   // Turn off relay 2
    Serial.println("Stop");    // Log stop command
    
  } 
  // Priority 2: Force Down - when at upper limit and down button pressed
  else if (digitalRead(LimitUp) == 0 && digitalRead(swDwn) == 0 && digitalRead(swUp) == 1) {
    digitalWrite(sig1, LOW);   // Turn off relay 1
    digitalWrite(sig2, HIGH);  // Turn on relay 2 (down direction)
    Serial.println("Down");    // Log down movement
    
  } 
  // Priority 3: Force Up - when at lower limit and up button pressed
  else if (digitalRead(LimitDown) == 0 && digitalRead(swDwn) == 1 && digitalRead(swUp) == 0) {
    digitalWrite(sig1, HIGH);  // Turn on relay 1 (up direction)
    digitalWrite(sig2, LOW);   // Turn off relay 2
    Serial.println("Up");      // Log up movement
    
  } 
  // Priority 4: Limit Protection - if either limit switch is activated
  else if (digitalRead(LimitUp) == 0 || digitalRead(LimitDown) == 0) {
    digitalWrite(sig1, LOW);   // Turn off relay 1
    digitalWrite(sig2, LOW);   // Turn off relay 2
    Serial.println("Max reached"); // Log limit reached
    
  }
  // Priority 5: Normal Up Movement - up button pressed, down button not pressed
  else if (digitalRead(swUp) == 0 && digitalRead(swDwn) == 1) {
    digitalWrite(sig1, HIGH);  // Turn on relay 1 (up direction)
    digitalWrite(sig2, LOW);   // Turn off relay 2
    Serial.println("Up");      // Log up movement
    
  } 
  // Priority 6: Normal Down Movement - down button pressed, up button not pressed
  else if (digitalRead(swDwn) == 0 && digitalRead(swUp) == 1) {
    digitalWrite(sig1, LOW);   // Turn off relay 1
    digitalWrite(sig2, HIGH);  // Turn on relay 2 (down direction)
    Serial.println("Down");    // Log down movement
    
  } 
  // Default: No movement - all buttons released or invalid combination
  else {
    digitalWrite(sig1, LOW);   // Turn off relay 1
    digitalWrite(sig2, LOW);   // Turn off relay 2
    Serial.println("Stop");    // Log stop state
  }
  
  // Note: The logic uses active LOW inputs (0 = pressed/activated)
  // due to the use of INPUT_PULLUP with normally open switches
}
