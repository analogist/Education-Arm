/*
  s2 892 90
  s2 552 0
  s2 188 -90
  y = 0.0045x - 2.4267

  s1 140 180
  s1 475 90
  s1 822 0
  y = -0.0046x + 3.7771

*/

#define l1 3.5
#define l2 8
#define DEBUGGING 0
#define EXSTART 1

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Potentiometer and controller definitions

#if DEBUGGING
long debugprinttime = 0;
#endif

#define pot1 A0
#define pot2 A1

#define down 2
#define right 5
#define up 4
#define left 3
#define button 6
#define allowbutton 11
#define buttonlight 12
long debouncetime_joy = 0;
long debounce_allowbutton = 0;
long debouncetime_button = 0; // button debouncing

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Internal states
bool ikmode = 0;
bool allowikmode = 0;
int pot1S, pot2S;
float th1, th2, x1, x2, y1, y2;
float qth1, qth2, qx, qy;

void setup() {
  #if DEBUGGING
  Serial.begin(9600);
  #endif

  // Controller setup
  pinMode(down, INPUT_PULLUP);
  pinMode(up, INPUT_PULLUP);
  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(button, INPUT_PULLUP);
  pinMode(allowbutton, INPUT_PULLUP);

  pinMode(buttonlight, OUTPUT);
  digitalWrite(buttonlight, LOW);

  // Motor setup
  AFMS.begin();  // create with the default frequency 1.6KHz
  motor1->setSpeed(100);
  motor1->run(FORWARD);
  motor1->run(RELEASE);
  motor2->setSpeed(100);
  motor2->run(FORWARD);
  motor2->run(RELEASE);
  
  // Feedback setup
  pot1S = analogRead(pot1);
  pot2S = analogRead(pot2);
  th1 = mapangle(pot1S, 1);
  th2 = mapangle(pot2S, 2);
  qth1 = th1;
  qth2 = th2;
  
  x1 = cos(th1)*l1;
  y1 = sin(th1)*l1;
  x2 = x1 + cos(th2+th1)*l2;
  y2 = y1 + sin(th2+th1)*l2;
  qx = x2;
  qy = y2;

  #if EXSTART
  qth1 = 1.12;
  qth2 = 1.57;
  moveto(qth1, qth2);
  #endif
}

void loop() {

  // Read pots
  pot1S = analogRead(pot1);
  pot2S = analogRead(pot2);
  th1 = mapangle(pot1S, 1);
  th2 = mapangle(pot2S, 2);
  x1 = cos(th1)*l1;
  y1 = sin(th1)*l1;
  x2 = x1 + cos(th2+th1)*l2;
  y2 = y1 + sin(th2+th1)*l2;

  // Read controller: drop to 0 when on
  bool downS = !digitalRead(down);
  bool upS = !digitalRead(up);
  bool leftS = !digitalRead(left);
  bool rightS = !digitalRead(right);
  bool buttonS = !digitalRead(button);
  bool allowbuttonS = !digitalRead(allowbutton);

  // TOGGLE FORWARD/INVERSE KINEMATICS
  if (allowbuttonS) {
    if ((millis() - debounce_allowbutton) > 500) {
      allowikmode = !allowikmode;
      debounce_allowbutton = millis();
    }
  }
  
  if (buttonS) {
    if (((millis() - debouncetime_button) > 500) && allowikmode) {
      ikmode = !ikmode;
      qx = x2;
      qy = y2;
      debouncetime_button = millis();
      if(ikmode){
        digitalWrite(buttonlight, HIGH);
      }
      else {
        digitalWrite(buttonlight, LOW);
      }
    }
  }
  
  // ---------- FORWARD KINEMATICS ---------------
  if (ikmode == 0) {
    if (leftS) {
      if((millis() - debouncetime_joy) > 50) {
        qth1 = qth1 - 0.01;
        if (qth1 < 0.17){ qth1 = 0.17; }
        debouncetime_joy = millis();
      }
    }
    else if (rightS) {
      if((millis() - debouncetime_joy) > 50) {
        qth1 = qth1 + 0.01;
        if (qth1 > 2.97) { qth1 = 2.97; }
        debouncetime_joy = millis();
      }
    }
    else if (downS) {
      if((millis() - debouncetime_joy) > 50) {
        qth2 = qth2 - 0.01;
        debouncetime_joy = millis();
      }
    }
    else if (upS) {
      if((millis() - debouncetime_joy) > 50) {
        qth2 = qth2 + 0.01;
        if (qth2 > 1.57){ qth2 = 1.57; }
        debouncetime_joy = millis();      
      }
    }
  }

  // ---------- INVERSE KINEMATICS -----------
  else if (ikmode == 1) {
    if (leftS) {
      if((millis() - debouncetime_joy) > 50) {
        qx = qx + 0.05;
        debouncetime_joy = millis();
      }
    }
    else if (rightS) {
      if((millis() - debouncetime_joy) > 50) {
        qx = qx - 0.05;
        debouncetime_joy = millis();
      }
    }
    else if (upS) {
      if((millis() - debouncetime_joy) > 50) {
        qy = qy - 0.05;
        debouncetime_joy = millis();
      }
    }
    else if (downS) {
      if((millis() - debouncetime_joy) > 50) {
        qy = qy + 0.05;
        debouncetime_joy = millis();      
      }
    }
  if(solveik(qx, qy) < 0) {
    qx = x2;
    qy = y2;
    qth1 = th1;
    qth2 = th2;
  }

  }

  // ===========MOVE========
  moveto(qth1, qth2);
  
  #if DEBUGGING
  if (millis() - debugprinttime > 200) {
    Serial.print("IK Mode: ");
    Serial.print(ikmode);
    Serial.print("\tAngles: ");
    Serial.print(th1);
    Serial.print(" (");
    Serial.print(qth1);
    Serial.print(")\t");
    Serial.print(th2);
    Serial.print(" (");
    Serial.print(qth2);
    Serial.print(")\t");
    Serial.print("[");
    Serial.print(x2);
    Serial.print(", ");
    Serial.print(y2);
    Serial.print("] ([");
    Serial.print(qx);
    Serial.print(", ");
    Serial.print(qy);
    Serial.println("])");
    debugprinttime = millis();
  }
  #endif
}

float mapangle(int value, char pot_no) {
  if (pot_no == 1) {
    return -0.0046*value + 3.7771;
  }
  else if (pot_no == 2) {
    return 0.0045*value - 2.4267;
  }
  else {
    #if DEBUGGING
    Serial.println("MAPANGLE() ERROR!");
    #endif
  }
}

void moveto(float qangle1, float qangle2) {
  long runcounter = millis();
  bool runstatus = 0;
  
  if (qangle1 >= 0.17 && qangle1 <= 2.97) {
    float q1diff = qangle1 - th1;
    int limspeed1 = min(50+(abs(q1diff)-0.01)*1000, 150);
    
    if (q1diff >= 0.01) {
      motor1->setSpeed(limspeed1);
      motor1->run(FORWARD);
      runstatus = 1;
    }
    else if (q1diff <= -0.01) {
      motor1->setSpeed(limspeed1);
      motor1->run(BACKWARD);
      runstatus = 1;
    }
  }

  if (qangle2 >= -1.57 && qangle2 <= 1.57) {
    float q2diff = qangle2 - th2;
    int limspeed2 = min(50+(abs(q2diff)-0.01)*2000, 150);
    
    if (q2diff >= 0.01) {
      motor2->setSpeed(limspeed2);
      motor2->run(BACKWARD);
      runstatus = 1;
    }
    else if (q2diff <= -0.01) {
      motor2->setSpeed(limspeed2);
      motor2->run(FORWARD);
      runstatus = 1;
    }
  }

  if (runstatus) {
    while((millis() - runcounter) < 10);
  }
  
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

int solveik(float qpos1, float qpos2) {
  float qangle1, qangle2, c2, s2, k1, k2;
  
  c2 = (sq(qpos1)+sq(qpos2)-sq(l1)-sq(l2))/(2*l1*l2);
  if ((1 - sq(c2)) >= 0) {
    s2 = sqrt(1 - sq(c2));
    qangle2 = atan2(s2, c2);

    k1 = l1 + l2*c2;
    k2 = l2*s2;

    qangle1 = atan2(qpos2, qpos1) - atan2(k2, k1);

    if (qangle1 >= 0.17 && qangle1 <= 2.97 && qangle2 >= -1.57 && qangle2 <= 1.57) {
      qth1 = qangle1;
      qth2 = qangle2;
      return 0;
    }
    else {
      return -1;
    }
  }
  else {
    return -1;
  }
}

