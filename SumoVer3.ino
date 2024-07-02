//Motor
#define PWM_PIN 9 
#define INA_PIN_L 5 
#define INB_PIN_L 6 
#define INA_PIN_R 10 
#define INB_PIN_R 11 

// Sensor พื้น
#define analogPin A5
#define digitalPin_L 2
#define digitalPin_R 3

//ปุ่ม เริ่มและเลือกโหมด
#define StartSumo 12
#define SelectMode 13

//ไฟแสดงสถานะ  LED 
#define Mode1 4  //สีแดง
#define Mode2 7  //สีเหลือง
#define Mode3 8 // สีขาว


// กำหนดพินที่เชื่อมต่อกับเซนเซอร์วัดระยะ
const int leftSensorPin = A0;
const int middleSensorPin = A1;
const int rightSensorPin = A2;
const int diagonally_left = A3;
const int diagonally_right = A4;

int delayValueL = 0;
int delayValueR = 0;

int StartValue = 0;
int SelectValue = 0;

//////////////////หมายเหตุ/////////////////////
//                                         //
//                                         //
//           ห้ามนำไปเผยแพร่ต่อเด็ดขาด          //
//                                        //
///////////////////////////////////////////


/////////////ส่วนนี้เอาไว้ปรับค่าต่างๆ//////////////////////
int speedMotor = 70;    // Speed Low 0-255 Max
int speedMotorRL = 100 ; //ความเร็วเลี้ยว 
int speedMotorBoost = 255; //ความเร็วพุ่งชน
int DistanceValue = 15;  //ระยะเซ็นเซอร์รอบทิศ
int DelayStartBot = 5000; // delay ก่อนเริ่มทำงาน
//////////////////////////////////////////////////

const int numReadings = 10;

int leftReadings[numReadings];
int middleReadings[numReadings];
int rightReadings[numReadings];
int diaLeftReadings[numReadings];
int diaRightReadings[numReadings];

int readIndex = 0;

int totalLeft = 0;
int totalMiddle = 0;
int totalRight = 0;
int totalDiaLeft = 0;
int totalDiaRight = 0;

int averageLeft = 0;
int averageMiddle = 0;
int averageRight = 0;
int averageDiaLeft = 0;
int averageDiaRight = 0;

unsigned long startTime = 0;  
bool startDelayDone = false;  

bool ifag = true;
void setup() {
  // เริ่มต้นการสื่อสารแบบอนุกรมเพื่อดูผลลัพธ์ใน Serial Monitor
  Serial.begin(9600);
  pinMode(INA_PIN_L, OUTPUT);
  pinMode(INB_PIN_L, OUTPUT);
  pinMode(INA_PIN_R, OUTPUT);
  pinMode(INB_PIN_R, OUTPUT);
  pinMode(Mode1, OUTPUT);
  pinMode(Mode2, OUTPUT);
  pinMode(Mode3, OUTPUT);
  pinMode(digitalPin_L, INPUT);
  pinMode(digitalPin_R, INPUT);
  pinMode(StartSumo,INPUT);
  pinMode(SelectMode,INPUT);


  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    leftReadings[thisReading] = 0;
    middleReadings[thisReading] = 0;
    rightReadings[thisReading] = 0;
    diaLeftReadings[thisReading] = 0;
    diaRightReadings[thisReading] = 0;
  }

  delay(3000);
}


/////////////////////////////////////////////////////////////////////////////////////
void loop() {
  static int lastButtonState = HIGH;  
  static int lastStartValue = StartValue;  
  int currentButtonState = digitalRead(StartSumo);  

  if (currentButtonState == LOW && lastButtonState == HIGH) {  
    StartValue = !StartValue; 
    delay(50); 

   
  }
  lastButtonState = currentButtonState;  

  static int lastButtonStateSelectMode = HIGH;  
  int currentButtonStateSelectMode = digitalRead(SelectMode);  

  if (currentButtonStateSelectMode == LOW && lastButtonStateSelectMode == HIGH) {  
    SelectValue = (SelectValue + 1) % 3;  
    delay(50);  // ดีเลย์เพื่อป้องกันการกดปุ่มซ้ำ (debounce)
  }
  lastButtonStateSelectMode = currentButtonStateSelectMode;  /
  // ตรวจสอบค่า SelectValue เพื่อเลือกโหมด
  switch (SelectValue) {
    case 0:
      Serial.println("Mode 1 Original");
      digitalWrite(Mode3,0);
      digitalWrite(Mode2,0);
      digitalWrite(Mode1, 1);
      break;
    case 1:
      Serial.println("Mode 2 Cyclone");
      digitalWrite(Mode1,0);
      digitalWrite(Mode2,1);
      break;
    case 2:
      Serial.println("Mode 3 Hey Stop!!");
      digitalWrite(Mode2,0);
      digitalWrite(Mode3,1);
      break;
  }
  
 // ตรวจสอบการเปลี่ยนแปลงของ StartValue
  if (StartValue == 0 && lastStartValue == 0) {
    Serial.println("Starting Bot!");  // แสดงข้อความเริ่มต้นการทำงานใน Serial Monitor
    switch (SelectValue) {
      case 0:
        while(ifag == true){
          Serial.print("Starting in 5 seconds...");
          delay(5000);
          ifag = false;
        }
        StartBotMode1();
        break;
      case 1:
        while(ifag == true){
          Serial.print("Starting in 5 seconds...");
          delay(5000);
          ifag = false;
        }
         StartBotMode2();
        break;
      case 2:
        while(ifag == true){
          Serial.print("Starting in 5 seconds...");
          delay(5000);
          ifag = false;
        }
        StartBotMode3();
        break;
    startDelayDone = false;  
  }
  lastStartValue = StartValue;  
}
}



void StartBotMode3(){
  Serial.print("Start MODE 3");
  
  int leftSensorValue = analogRead(leftSensorPin);
  int middleSensorValue = analogRead(middleSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);
  int diaLeftSensorValue = analogRead(diagonally_left);
  int diaRightSensorValue = analogRead(diagonally_right);

  totalLeft = totalLeft - leftReadings[readIndex] + leftSensorValue;
  totalMiddle = totalMiddle - middleReadings[readIndex] + middleSensorValue;
  totalRight = totalRight - rightReadings[readIndex] + rightSensorValue;
  totalDiaLeft = totalDiaLeft - diaLeftReadings[readIndex] + diaLeftSensorValue;
  totalDiaRight = totalDiaRight - diaRightReadings[readIndex] + diaRightSensorValue;

  leftReadings[readIndex] = leftSensorValue;
  middleReadings[readIndex] = middleSensorValue;
  rightReadings[readIndex] = rightSensorValue;
  diaLeftReadings[readIndex] = diaLeftSensorValue;
  diaRightReadings[readIndex] = diaRightSensorValue;

  readIndex = (readIndex + 1) % numReadings;

  int averageLeft = totalLeft / numReadings;
  int averageMiddle = totalMiddle / numReadings;
  int averageRight = totalRight / numReadings;
  int averageDiaLeft = totalDiaLeft / numReadings;
  int averageDiaRight = totalDiaRight / numReadings;

  const float voltageConversionFactor = 5.0 / 1023.0;
  const float distanceFactor = 12.08;
  const float distanceExponent = -1.058;
  
  float leftVoltage = averageLeft * voltageConversionFactor;
  float middleVoltage = averageMiddle * voltageConversionFactor;
  float rightVoltage = averageRight * voltageConversionFactor;
  float diaLeftVoltage = averageDiaLeft * voltageConversionFactor;
  float diaRightVoltage = averageDiaRight * voltageConversionFactor;
  
  int leftDistance = distanceFactor * pow(leftVoltage, distanceExponent);
  int middleDistance = distanceFactor * pow(middleVoltage, distanceExponent);
  int rightDistance = distanceFactor * pow(rightVoltage, distanceExponent);
  int diaLeftDistance = distanceFactor * pow(diaLeftVoltage, distanceExponent);
  int diaRightDistance = distanceFactor * pow(diaRightVoltage, distanceExponent);

  if (leftDistance < DistanceValue && diaLeftDistance > DistanceValue && middleDistance > DistanceValue && diaRightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(leftDistance);
    Serial.println(" Turning left");
    tleft(1);
  } else if (leftDistance > DistanceValue && diaLeftDistance < DistanceValue && middleDistance < DistanceValue && diaRightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(diaLeftDistance);
    Serial.println(" Dia_Turning left");
    forward_boost();
  } else if (leftDistance > DistanceValue && diaLeftDistance < DistanceValue && middleDistance < DistanceValue && diaRightDistance < DistanceValue && rightDistance > DistanceValue) {
    Serial.println(" Center");
    Serial.print(middleDistance);
    forward_boost();
  } else if (leftDistance > DistanceValue && diaLeftDistance > DistanceValue && middleDistance > DistanceValue && diaRightDistance < DistanceValue && rightDistance < DistanceValue) {
    Serial.print(diaRightDistance);
    Serial.println(" Turning Dia_right");
    forward_boost();
  } 
  
  else if (leftDistance > DistanceValue && diaLeftDistance > DistanceValue && middleDistance > DistanceValue && diaRightDistance > DistanceValue && rightDistance < DistanceValue) {
    Serial.print(rightDistance);
    Serial.println(" Turning right ");
    tright(1);
  }

  // ตรวจสอบเซ็นเซอร์พื้น
  int analogValue = analogRead(analogPin);
  int digitalValue_L = digitalRead(digitalPin_L);
  int digitalValue_R = digitalRead(digitalPin_R);
  float voltage = analogValue * voltageConversionFactor;

  if (digitalValue_L == 1 || digitalValue_R == 1) {
    Serial.println(" backward++++");
    backward();
    delay(500);
    tright(200);
    Serial.println(" T_Right<<<<");
  } else {
    Serial.println(" forward----");
    forward();
  }

  delay(50);
}

void StartBotMode2() {
  // โค้ดสำหรับการเริ่มต้นบอทในโหมดที่ 2
  Serial.println("Starting Bot in Mode 2");
  

  int leftSensorValue = analogRead(leftSensorPin);
  int middleSensorValue = analogRead(middleSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);
  int dia_leftSensorValue = analogRead(diagonally_left);
  int dia_rightSensorValue = analogRead(diagonally_right);
  
  float leftVoltage = leftSensorValue * (5.0 / 1023.0);
  float middleVoltage = middleSensorValue * (5.0 / 1023.0);
  float rightVoltage = rightSensorValue * (5.0 / 1023.0);
  float dia_leftVoltage = dia_leftSensorValue * (5.0 / 1023.0);
  float dia_rightVoltage = dia_rightSensorValue * (5.0 / 1023.0);
      
  int leftDistance = 12.08 * pow(leftVoltage, -1.058);
  int middleDistance = 12.08 * pow(middleVoltage, -1.058);
  int rightDistance = 12.08 * pow(rightVoltage, -1.058);
  int dia_leftDistance = 12.08 * pow(dia_leftVoltage, -1.058);
  int dia_rightDistance = 12.08 * pow(dia_rightVoltage, -1.058);

/*
  Serial.print("L : Distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");
  
  Serial.print("DL : Distance: ");
  Serial.print(dia_leftDistance);
  Serial.println(" cm");
  
  Serial.print("C : Distance: ");
  Serial.print(middleDistance);
  Serial.println(" cm");
    
  Serial.print("DR : Distance: ");
  Serial.print(dia_rightDistance);
  Serial.println(" cm");
  
  Serial.print("R : Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");
*/
/////////////////////////////////////////////////////////////////////////////////////////////


  int analogValue = analogRead(analogPin);

  int digitalValue_L = digitalRead(digitalPin_L);
  int digitalValue_R = digitalRead(digitalPin_R);


  float voltage = analogValue * (5.0 / 1023.0);
/*
  Serial.print("V - Digital Value_L: ");
  Serial.println(digitalValue_L);
  Serial.print("V - Digital Value_R: ");
  Serial.println(digitalValue_R);
*/
 
////////////////////////////////////////////////////////////////////////
  if (leftDistance < DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(leftDistance);
    Serial.println(" Turning left");
    tleft(20);
  } 
  else if (leftDistance > DistanceValue && dia_leftDistance < DistanceValue && middleDistance > DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(dia_leftDistance);
    Serial.println(" Dia_Turning left");
     tleft(10);
  } 
  
  else if (leftDistance > DistanceValue && dia_leftDistance < DistanceValue && middleDistance < DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(dia_leftDistance);
    Serial.println(" Dia_Turning left");
    forward_boost();
  } 
  else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance < DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.println(" Center");
    Serial.print(middleDistance);
    forward_boost();
  } else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance < DistanceValue && rightDistance < DistanceValue) {
    Serial.print(dia_rightDistance);
    Serial.println(" Turning Dia_right");
    forward_boost();
  } 
  else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance < DistanceValue && rightDistance > DistanceValue) {
    Serial.print(dia_rightDistance);
    Serial.println(" Turning Dia_right");
    tright(10);
  } 
  
  else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance > DistanceValue && rightDistance < DistanceValue) {
    Serial.print(rightDistance);
    Serial.println(" Turning right ");
    tright(20);
  }
  
  if (digitalValue_L == 1 || digitalValue_R == 1){
    
    Serial.println("backward++++");
    backward();
    delay(200);
    tright(100);
    Serial.println("T_Right<<<<");
    
   
  }
  else{
    Serial.println("forward----");
    forward();
     
  }

  delay(10);
}

void StartBotMode1() {
  // โค้ดสำหรับการเริ่มต้นบอทในโหมดที่ 3
  Serial.println("Starting Bot in Mode 1");
  
  int leftSensorValue = analogRead(leftSensorPin);
  int middleSensorValue = analogRead(middleSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);
  int dia_leftSensorValue = analogRead(diagonally_left);
  int dia_rightSensorValue = analogRead(diagonally_right);
  
  float leftVoltage = leftSensorValue * (5.0 / 1023.0);
  float middleVoltage = middleSensorValue * (5.0 / 1023.0);
  float rightVoltage = rightSensorValue * (5.0 / 1023.0);
  float dia_leftVoltage = dia_leftSensorValue * (5.0 / 1023.0);
  float dia_rightVoltage = dia_rightSensorValue * (5.0 / 1023.0);
      
  int leftDistance = 12.08 * pow(leftVoltage, -1.058);
  int middleDistance = 12.08 * pow(middleVoltage, -1.058);
  int rightDistance = 12.08 * pow(rightVoltage, -1.058);
  int dia_leftDistance = 12.08 * pow(dia_leftVoltage, -1.058);
  int dia_rightDistance = 12.08 * pow(dia_rightVoltage, -1.058);

/*
  Serial.print("L : Distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");
  
  Serial.print("DL : Distance: ");
  Serial.print(dia_leftDistance);
  Serial.println(" cm");
  
  Serial.print("C : Distance: ");
  Serial.print(middleDistance);
  Serial.println(" cm");
    
  Serial.print("DR : Distance: ");
  Serial.print(dia_rightDistance);
  Serial.println(" cm");
  
  Serial.print("R : Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");
*/
/////////////////////////////////////////////////////////////////////////////////////////////


  int analogValue = analogRead(analogPin);

  int digitalValue_L = digitalRead(digitalPin_L);
  int digitalValue_R = digitalRead(digitalPin_R);


  float voltage = analogValue * (5.0 / 1023.0);
/*
  Serial.print("V - Digital Value_L: ");
  Serial.println(digitalValue_L);
  Serial.print("V - Digital Value_R: ");
  Serial.println(digitalValue_R);
*/
 
////////////////////////////////////////////////////////////////////////
 

  if (leftDistance < DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(leftDistance);
    Serial.println(" Turning left");
    tleft(200);
  } 
  else if (leftDistance > DistanceValue && dia_leftDistance < DistanceValue && middleDistance > DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(dia_leftDistance);
    Serial.println(" Dia_Turning left");
     tleft(50);
  } 
  
  else if (leftDistance > DistanceValue && dia_leftDistance < DistanceValue && middleDistance < DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.print(dia_leftDistance);
    Serial.println(" Dia_Turning left");
    forward_boost();
  } 
  else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance < DistanceValue && dia_rightDistance > DistanceValue && rightDistance > DistanceValue) {
    Serial.println(" Center");
    Serial.print(middleDistance);
    forward_boost();
  } else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance < DistanceValue && rightDistance < DistanceValue) {
    Serial.print(dia_rightDistance);
    Serial.println(" Turning Dia_right");
    forward_boost();
  } 
  else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance < DistanceValue && rightDistance > DistanceValue) {
    Serial.print(dia_rightDistance);
    Serial.println(" Turning Dia_right");
    tright(50);
  } 
  
  else if (leftDistance > DistanceValue && dia_leftDistance > DistanceValue && middleDistance > DistanceValue && dia_rightDistance > DistanceValue && rightDistance < DistanceValue) {
    Serial.print(rightDistance);
    Serial.println(" Turning right ");
    tright(200);
  }
  
  if (digitalValue_L == 1 || digitalValue_R == 1){
    
    Serial.println("backward++++");
    backward();
    delay(200);
    tright(100);
    Serial.println("T_Right<<<<");
    
   
  }
  else{
    Serial.println("forward----");
    forward();
     
  }

  delay(10);
  
}

void Stop(){
  analogWrite(PWM_PIN, speedMotor);
  digitalWrite(INA_PIN_L, HIGH);
  digitalWrite(INB_PIN_L, HIGH);
  digitalWrite(INA_PIN_R, HIGH);
  digitalWrite(INB_PIN_R, HIGH);
}
void forward() {
  analogWrite(PWM_PIN, speedMotor);
  digitalWrite(INA_PIN_L, HIGH);
  digitalWrite(INB_PIN_L, LOW);
  digitalWrite(INA_PIN_R, HIGH);
  digitalWrite(INB_PIN_R, LOW);
}

void forward_boost() {
  analogWrite(PWM_PIN, speedMotorBoost);
  digitalWrite(INA_PIN_L, HIGH);
  digitalWrite(INB_PIN_L, LOW);
  digitalWrite(INA_PIN_R, HIGH);
  digitalWrite(INB_PIN_R, LOW);
}

void backward() {
  analogWrite(PWM_PIN, speedMotor);
  digitalWrite(INA_PIN_L, LOW);
  digitalWrite(INB_PIN_L, HIGH);
  digitalWrite(INA_PIN_R, LOW);
  digitalWrite(INB_PIN_R, HIGH);
}

void tleft(int delayValueL){
  analogWrite(PWM_PIN, speedMotorRL);
  digitalWrite(INA_PIN_L, LOW);
  digitalWrite(INB_PIN_L, HIGH);
  digitalWrite(INA_PIN_R, HIGH);
  digitalWrite(INB_PIN_R, LOW);
  delay(delayValueL);

}

void tright(int delayValueR){
  analogWrite(PWM_PIN, speedMotorRL);
  digitalWrite(INA_PIN_L, HIGH);
  digitalWrite(INB_PIN_L, LOW);
  digitalWrite(INA_PIN_R, LOW);
  digitalWrite(INB_PIN_R, HIGH);
  delay(delayValueR);
}
