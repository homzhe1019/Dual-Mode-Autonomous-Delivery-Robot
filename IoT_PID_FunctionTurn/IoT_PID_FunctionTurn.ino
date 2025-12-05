  #include <Wire.h> //control data output input
  #include <Adafruit_SSD1306.h> //control display OLED
  #include "BluetoothSerial.h" // bluetooth signal
  #include <ESP32Servo.h>

  //Setting of OLED
  #define OLED_WIDTH 128
  #define OLED_HEIGHT 64
  #define OLED_RESET -1 // no reset pin
  Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET); //create a screen, and display the content in the () // &wire - spscific 'road' for data transfer

  //Setting bluetooth object
  BluetoothSerial ESP32_BT; 

  Servo myServo; 
  const int SERVO_PIN = 19; 
  bool servoState = false;  

  // Pin of IR Line Tracking Sensor
  const int IR_PINS[] = {36, 39, 34, 35, 4}; //definition the pin at esp 32
  const int NUM_SENSORS = 5; //number of IR sensor

  // Pin of Motor Driver
  const int DIN_LF = 27; 
  const int DIN_LB = 26;
  const int PWM_L = 14;

  const int DIN_RF = 25;
  const int DIN_RB = 33;
  const int PWM_R = 32;

  // Pin of ultrasonic
  const int TRIG_PIN = 5;  // Trigger Pin
  const int ECHO_PIN = 18; // Echo Pin
  const int OBJECT_DIST = 15; // Stop distance in cm

  // Logic and variable setting
  const int Black_line = LOW;
  bool autoMode = false;
  String state = "";
  String lastState = "";
  int s[5];
  char cmd;
  long duration;
  int distanceCm;
  int lastValidSide = 0; 
  const int weights[5] = {-7, -3, 0, 3, 7};  //know car position (-ve and +ve)
  const int LOST_VALUE = 10; 
  const int TURN_DETECTED = 20;

  //PID setting
  float Kp = 23.0;  // responce degree higher Kp higher correction
  float Ki = 0.0;   // integrate(set 0 first)
  float Kd = 6.0;  // predict direction(prevent shaking) 

  float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
  float previous_error = 0;

  //speed setting
  int baseSpeed = 110; 
  int maxSpeed = 200;  


  // Fuction declaration
  void moveForward();
  void moveBack();
  void turnLeft();
  void turnRight();
  void spinLeft();
  void spinRight();
  void stopMotor();
  void updateScreen(String text); 
  void MotorSpeed(int leftSpeed, int rightSpeed); 
  void toggleServo();
  int getDistance();
  void autoLineFollowPID(); 
  int calculateSensorError();

  void setup() {

    Serial.begin(115200); // send data to seriall monitor
    Serial.println("IR Sensor Test Started...");
    Serial.println("Please open the Serial Monitor at upper right");
    
    // switch ob bluetooth
    ESP32_BT.begin("IoT_ESP32");
    Serial.println("Bluetooth Started! Ready to pair");

    // IR Line Sensor setup
    for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(IR_PINS[i], INPUT); // pin mode set as input( esp 32 input data from IR sensor)
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
  
    myServo.setPeriodHertz(50);    // 标准 50hz 舵机
    myServo.attach(SERVO_PIN, 500, 2400); // 🔗 绑定引脚和脉宽范围
    myServo.write(0); // 🏁 初始位置归零 (0度)
    
    // Motor Driver setup
    pinMode(DIN_LF, OUTPUT); pinMode(DIN_LB, OUTPUT);
    pinMode(DIN_RF, OUTPUT); pinMode(DIN_RB, OUTPUT);
    pinMode(PWM_L, OUTPUT); pinMode(PWM_R, OUTPUT); // pin mode set as output (esp 32 control voltage for motor)
    stopMotor();
    delay(2000); // pause 2s for handware stable
    
    // Ultrasonic setup
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // OLED setup
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // esp32 supply voltage and address 0x3C
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Will not continue proceed, loop forever and stop here
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); // word start display position
    display.println("System Ready"); //store in buffer first
    display.display(); // load and refresh the content in buffer
    delay(2000);
    display.clearDisplay(); 
    display.display();

  }

  void loop() {

    // Bluetooth
    handleBluetooth();

    // Get current distance
    int currentDist = getDistance();
    // determine have something block or not
    bool objectDetected = (currentDist > 0 && currentDist < OBJECT_DIST);
    
    // Auto or manual mode
    if (autoMode == true){
      if (objectDetected){
        stopMotor();
      }
      else{
        autoLineFollowPID(); 
      }
    }


    if(state != lastState){
      updateScreen(state);
      lastState = state;
    }  

    if(!autoMode){
      switch (cmd) {
          case 'F':
            if (objectDetected){
             stopMotor();
            }
            else{
              moveForward();
            }
            break;
          case 'B':
            moveBack();
            break;
          case 'L':
            spinLeft();
            break;
          case 'R':
            spinRight();
            break;

          case 'Y':
             toggleServo(); // 👉 去执行舵机动作
             cmd = ' '; // 🧹 清空指令，防止一直重复执行
             break;
          default:
            stopMotor();
            break;
          }
      
    }

  }

  void handleBluetooth(){
    if(ESP32_BT.available()){
      cmd = ESP32_BT.read();

      if(cmd == 'A'){
        autoMode = true;
        Serial.println("MODE: AUTO");
        state = "AUTO";
      }

      else if(cmd == 'M'){
        autoMode = false;
        Serial.println("MODE: Manual");
        state = "MANUAL";
      }

    }
  }

  int calculateSensorError() {
  int sensorVal[5]; // record the IR result in [a b c d e]
  int activeSensors = 0; //store how many antive sensor(black)
  long weightedSum = 0; // for calculation to balance
  

  for (int i = 0; i < NUM_SENSORS; i++) {
    
    // form the result in [a b c d e], then calculate the weight
    if (digitalRead(IR_PINS[i]) == Black_line) {
      sensorVal[i] = 1; 
      weightedSum += weights[i]; // (the value more close to zero, less correction)
      activeSensors++;
    } else {
      sensorVal[i] = 0;
    }
  }
  
  for(int i=0; i<5; i++) s[i] = digitalRead(IR_PINS[i]); //store at global 

  if (s[0] == Black_line && s[1] == Black_line && s[2] == Black_line) { //90° Left
     return -TURN_DETECTED; 
  }

  if (s[2] == Black_line && s[3] == Black_line && s[4] == Black_line) { //90° Right
     return TURN_DETECTED; 
  }

  if (activeSensors == 0) { // when lost(no line detected)
    if (previous_error < 0) return -LOST_VALUE; //spin to find line
    else return LOST_VALUE; 
  }

  return weightedSum / activeSensors; //calculate the average error (lower error lower correction)
}

void autoLineFollowPID() {
  error = calculateSensorError();

  if (error == -TURN_DETECTED) { 
    spinLeft(); 
    delay(180);  // turn without IR

    long startTime = millis(); // start record time (PID not handle now)
    while (digitalRead(IR_PINS[2]) != Black_line) { // while long time not found line
      if(millis() - startTime > 2000) break;        // continue below
    }

    stopMotor();  // if found center line, take a rest
    delay(100); 
    
    error = 0; previous_error = 0; D = 0; I = 0; // reset variable
    return;  // give back PID handle
  } 

   else if (error == TURN_DETECTED) {
    spinRight();
    delay(180); 

    long startTime = millis();
    while (digitalRead(IR_PINS[2]) != Black_line) {
      if(millis() - startTime > 2000) break;
    }

    stopMotor();
    delay(100);
    
    error = 0; previous_error = 0; D = 0; I = 0;
    return;
  }


  
  if (error == -LOST_VALUE) { //lost line, spin to find line
    spinLeft(); 
    previous_error = -LOST_VALUE;
    return;
  } else if (error == LOST_VALUE) {
    spinRight();
    previous_error = LOST_VALUE;
    return;
  }

  if (abs(previous_error) == LOST_VALUE && abs(error) < LOST_VALUE) { //prevent d changes too large 
    error = 0; 
    D = 0;  
    previous_error = 0; 
  }

  P = error; //Propertional - the higher error the higher correction
  I = I + error; //no use in this project
  if (error == 0) I = 0;
  D = error - previous_error; // dydx -  
  
  PID_value = (Kp * P) + (Ki * I) + (Kd * D); //calculate correction value
  
  previous_error = error; // store for next time calculate D

  int leftMotorSpeed = baseSpeed - PID_value; // use pid value to adjust motor speed
  int rightMotorSpeed = baseSpeed + PID_value;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed); //limit the speed between min and max
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  MotorControlPID(leftMotorSpeed, rightMotorSpeed);
}

 
 void MotorControlPID(int speedL, int speedR) { //get right/left speed from above
  if (speedL >= 0) {
    digitalWrite(DIN_LF, HIGH); digitalWrite(DIN_LB, LOW); // starting use pid value correct direction
    analogWrite(PWM_L, speedL);
  } else {
    digitalWrite(DIN_LF, LOW); digitalWrite(DIN_LB, HIGH); // if correction too much, reverse back
    analogWrite(PWM_L, abs(speedL));
  }

  if (speedR >= 0) {
    digitalWrite(DIN_RF, HIGH); digitalWrite(DIN_RB, LOW);
    analogWrite(PWM_R, speedR);
  } else {
    digitalWrite(DIN_RF, LOW); digitalWrite(DIN_RB, HIGH);
    analogWrite(PWM_R, abs(speedR));
  }
}


  void updateScreen(String text) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(50, 25);
    display.println(text);
    display.display();
    }

  void MotorSpeed(int leftSpeed, int rightSpeed) {
    analogWrite(PWM_L, leftSpeed);
    analogWrite(PWM_R, rightSpeed);
  }


  void moveForward(){
    digitalWrite(DIN_LF, HIGH); digitalWrite(DIN_LB, LOW);
    digitalWrite(DIN_RF, HIGH); digitalWrite(DIN_RB, LOW);
    MotorSpeed(150, 150);
  }

  void moveBack(){
    digitalWrite(DIN_LF, LOW); digitalWrite(DIN_LB, HIGH);
    digitalWrite(DIN_RF, LOW); digitalWrite(DIN_RB, HIGH);
    MotorSpeed(150, 150);
  }

  void turnLeft(){
    digitalWrite(DIN_LF, LOW); digitalWrite(DIN_LB, LOW);
    digitalWrite(DIN_RF, HIGH); digitalWrite(DIN_RB, LOW);
    MotorSpeed(150, 0);
  }

  void turnRight(){
    digitalWrite(DIN_LF, HIGH); digitalWrite(DIN_LB, LOW);
    digitalWrite(DIN_RF, LOW); digitalWrite(DIN_RB, LOW);
    MotorSpeed(0, 150);
  }

  void spinLeft(){
    digitalWrite(DIN_LF, LOW); digitalWrite(DIN_LB, HIGH);
    digitalWrite(DIN_RF, HIGH); digitalWrite(DIN_RB, LOW);
    MotorSpeed(130, 130);
  }

  void spinRight(){
    digitalWrite(DIN_LF, HIGH); digitalWrite(DIN_LB, LOW);
    digitalWrite(DIN_RF, LOW); digitalWrite(DIN_RB,HIGH);
    MotorSpeed(130, 130);
  }

  void stopMotor(){
    digitalWrite(DIN_LF, LOW); digitalWrite(DIN_LB, LOW);
    digitalWrite(DIN_RF, LOW); digitalWrite(DIN_RB, LOW);
    MotorSpeed(0, 0);
  }

  int getDistance(){
    digitalWrite(TRIG_PIN, LOW); //set low first
    delayMicroseconds(2);  
    digitalWrite(TRIG_PIN, HIGH); //then set high and shoot
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW); //set low again prepair for accept

    duration = pulseIn(ECHO_PIN, HIGH, 8000); //determine the duration form generate to receive
    distanceCm = duration * 0.034 / 2; //convert the distance in cm

    if (distanceCm == 0 || distanceCm > 400) {
    return 999; // when invalid or too far
    }
    
    return distanceCm;
  }

  void toggleServo() {
  // 🔄 这是一个开关逻辑 (Toggle)
  if (servoState == false) {
    // 🚀 当前是 0度，收到指令 -> 转到 90度
    myServo.write(90);
    servoState = true; // 📝 记住现在的状态是 90度
  } else {
    // ⬇️ 当前是 90度，收到指令 -> 回到 0度
    myServo.write(0);
    servoState = false; // 📝 记住现在的状态是 0度
  }
  delay(300); // ⏳稍微防抖一下，也给舵机一点时间反应
}

