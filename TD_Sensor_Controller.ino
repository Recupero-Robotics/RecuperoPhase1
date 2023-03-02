///////////
//HEADERS//
///////////

//Robot headers
#include <SPI.h>
#include <SD.h>
#include <Math.h>
#include <Encoder.h>
#include <TimeLib.h>
#include <Stepper.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//HR sensor header
#define heartratePin A9
#include <DFRobot_Heartrate.h>
DFRobot_Heartrate heartrate(DIGITAL_MODE);

////////////////////////
//Microcontroller Pins//
////////////////////////

//Servo
int servoOutputPin = A22;

//Pushbutton
int buttonPin = 29;

//Handle/accessory
// When handle is changed we can still use these inputs
int fsrPin1 = A0; // Force sensor raw input 1/accessory data
int fsrPin2 = A1;
int fsrPin3 = A2;
int fsrPin4 = A3;

//GSR Sensor
int GSRPin = A6;

//Potentiometer
int pot_pin=A21;

//Stepper
const int stepsPerRevolution = 200;  // steps per rev of motor
Stepper myStepper(stepsPerRevolution, 5, 6, 7, 8);

//Encoder
Encoder encoder(0, 1);

///////////////////////////
//Variable initialization//
///////////////////////////

//SD Card and Time
File dataFile;
time_t timeStamp;
time_t trialTime;
String trialLabel = "UnknownApp";
String userId = "dummy_R";
char dirName[64];
char trialLabelBuffer[64];
unsigned int currentTime = 0;
unsigned int previousTime = 0;
unsigned int sampleTime = 0;
int calibrationFlag = 0;
int rightState = 1; //1 for right handed, 0 for left handed

//Serial
int baudRate = 115200;

//Pinch handle
float pot_val=0;
float pinch_angle=0; //Current pinch angle
float set_angle=90; //Angle to go to
int stepper_speed = 30; //Resistance of stepper

//Raw Robot State
int rawEncoder = 0; //Raw encoder data
float robotAngle = 0.0; // angle of robot
float desiredAngle = 0.0; //Set point for the robot
float pinchAngle = 0.0; //Angle of pinch
int pushButton = 0; // button state for single pushbutton
int buttonArray[10];
int GSR = 0; // Galvanic skin response magnitude
float humanResistance = 0; //Ohm
float microSiemens = 0; //Microsiemens
int siemensArray[60];
int heartrateArray[60];
int calibrationValueGSR = 45;
int calibrationValueHR = 70;
int HR = 0; // heart rate
int oldHR = 0;

//Mapped Robot States
int theta1 = 0; //-135 to 135 degrees, mapped 000 to 270 for serial
int theta2 = 0; // 0 to 180 degrees, mapped 0-90 from 0-90 and from 90-180
int force1 = 0; //Newtons
int button1 = 0; //0 (Pressed) or 1 (Unpressed)
int GSR1 = 0; //mapped 0 (Bored), 1 (Flow), 2 (Frustrated) for serial
int HR1 = 0; //mapped 0 (Bored), 1 (Flow), 2 (Frustrated) for serial

//Serial input State
char startState[2] = {'d'}; //(1 byte) tells the Arduino to start or stop recording to SD card
int controlMode = 0; //(1 byte) sets the mode of operation/control law for the TD or handle
int perfGain = 0; //(3 bytes) sets the stiffness of the controller
int setPoint = 0; //(3 bytes) is a placeholder for any data from the game which could feed back into the controller

//Servo amp
double servoOutput = .5; // 0 to 1, 0 is full power left, 1 is full power right, 0.5 is no output
double oldServoOutput = .5;
double servoOutputFilt = .5;
float controlGain = 1;
double Kp = 0.008;
double Ki = 0.000025;
double Kd = 0;
double error_last = 0.0;
double deltaT = 0.07;
double omegaFiltOld = 0.0;
double omega;
double error;
double total_error_max = 1500.0;
double total_error = 0;

//Pushbutton
int buttonState = 0; // variable for reading the pushbutton status, pressed is LOW
elapsedMillis gripTime = 0;
double gripThreshold = 200; //milliseconds

//Handle force sensors
int fsrReading1 = 0; //uncalibrated
int fsrReading2 = 0;
int fsrReading3 = 0;
int fsrReading4 = 0;

float fsr1 = 0; //calibrated
float fsr2 = 0;
float fsr3 = 0;
float fsr4 = 0;
float net_force = 0; //sum of handle forces

//IMUs
MPU6050 shouldergyro(0x68, &Wire1); 
MPU6050 sternumgyro(0x68); 
int16_t ax_shoulder, ay_shoulder, az_shoulder;
int16_t gx_shoulder, gy_shoulder, gz_shoulder;
int16_t ax_sternum, ay_sternum, az_sternum;
int16_t gx_sternum, gy_sternum, gz_sternum;

#define OUTPUT_READABLE_ACCELGYRO

//Bio sensors
//int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore. default "550" value.
//PulseSensorPlayground pulseSensor;
//int sensorValue = 0;

String startTime = "";
bool initialized = false;

void setup() {

  //Initialize SD Card
  SD.begin(BUILTIN_SDCARD);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card not initialized");
    while (true);
  }

  //Connect IMU
  Wire.begin();
  Wire1.begin();
  Serial.println("Initializing I2C devices...");
  shouldergyro.initialize();
  sternumgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(shouldergyro.testConnection() ? "Shoulder connection successful" : "Shoulder connection failed");
  Serial.println(sternumgyro.testConnection() ? "Sternum connection successful" : "Sternum connection failed");
//  if(!shouldergyro.testConnection() || !sternumgyro.testConnection()){
//    Serial.println("Check IMU connection");
//    while (true);
//  }

  //Change accel/gyro offset values
  shouldergyro.setXAccelOffset(-2625);
  shouldergyro.setYAccelOffset(2091);
  shouldergyro.setZAccelOffset(1470);
  shouldergyro.setXGyroOffset(99);
  shouldergyro.setYGyroOffset(-34);
  shouldergyro.setZGyroOffset(-27);
  sternumgyro.setXAccelOffset(-6547);
  sternumgyro.setYAccelOffset(396);
  sternumgyro.setZAccelOffset(720);
  sternumgyro.setXGyroOffset(115);
  sternumgyro.setYGyroOffset(-23);
  sternumgyro.setZGyroOffset(-13);

  //Initialize encoder
  encoder.write(0); //zero out encoder

  //Initialize pushbutton
  pinMode(buttonPin, INPUT_PULLUP);   //pull up so that reads high when switch not pressed
  delayMicroseconds(10);

  //Initialize Stepper
  myStepper.setSpeed(30); //Changes resistance

  //Initialize HR sensor
//  pulseSensor.analogInput(HRPin);
//  pulseSensor.setThreshold(Threshold);
//  pulseSensor.begin();

  //start serial comm
  Serial.begin(baudRate);
}

//Used to create file
void createDataFile(String fileType){
  //Check if patient directory exists and create if not
  userId.toCharArray(dirName, userId.length()+1);
  if (!SD.exists(dirName)){
    SD.mkdir(dirName);
  }

  if(fileType != "CalibrationValue"){
    //Creates first possible text file name
    int trialNum = 1;
    trialLabel = userId + "/" + fileType + String(trialNum) + ".txt";
    trialLabel.toCharArray(trialLabelBuffer, trialLabel.length() + 1);
  
    //Creates new file based on extisting trial files
    while(SD.exists(trialLabelBuffer)){
      trialNum += 1;
      trialLabel = userId + "/" + fileType + String(trialNum) + ".txt";
      trialLabel.toCharArray(trialLabelBuffer, trialLabel.length() + 1);
    }
  }
  else if (fileType == "CalibrationGSR"){
    //Creates text file name
    trialLabel = userId + "/" + fileType + ".txt";
    trialLabel.toCharArray(trialLabelBuffer, trialLabel.length() + 1);
  }
  else if (fileType == "CalibrationHR"){
    //Creates text file name
    trialLabel = userId + "/" + fileType + ".txt";
    trialLabel.toCharArray(trialLabelBuffer, trialLabel.length() + 1);
  }

}

void readSerialInput() {
  String incomingString; //String from serial
  char incomingCharArray[64]; //Full 64 byte buffer to store
  if (Serial.available() > 0) {
    incomingString = Serial.readStringUntil('\n'); //Read the entire buffer as a String
    //PARSE INCOMING DATA
    // format: "startState,controlMode,performanceGain,setAngle"
    // startState (1 byte) tells the Arduino to start or stop recording to SD card
    // controlMode (1 byte) sets the mode of operation/control law for the TD or handle
    // performanceGain (3 bytes) sets the stiffness of the controller
    // setAngle (3 bytes) is a placeholder for any data from the game which could feed back into the controller

    //Convert String to a 64byte char array
    incomingString.toCharArray(incomingCharArray, 64);

    char * incomingIdx; //Index for each data point

    //Convert first entry (startState) to char array
    incomingIdx = strtok(incomingCharArray, ",");
    strcpy(startState, incomingIdx);

    //If state is e for end, don't do anything
    if(startState[0] == 'e'){
      initialized = false;
      setPoint = 0;
      return;
    }

    //If a start state has not been given
    if(!initialized){
      //s for start recording; logs user ID and unity start time while creating data file for user
      if (startState[0] == 's') {
        initialized = true;
  
        userId = strtok(NULL, ",");
        if(userId == ""){
          userId = "Unknown_User";
        }

        //Check for left or right handed
        int length_string = userId.length();
        if(userId[length_string-1] == 'L'){
          rightState = 0;
        }
        else{
          rightState = 1;
        }
        
        startTime = strtok(NULL, ",");

        trialLabel = strtok(NULL, ",");
        if(trialLabel == ""){
          trialLabel = "Unknown_App";
        }

        //Create file
        createDataFile(trialLabel);

        //Write the Unity Time to the data file
        dataFile = SD.open(trialLabelBuffer, FILE_WRITE);
        if (dataFile) {
          dataFile.print("Unity Time:   ");
          dataFile.println(startTime); //1 Byte //AnalogB2
          dataFile.close();
        }
      }

      //c for calibration; does the same as s but adds an aditional calibration phase at the end
      if (startState[0] == 'c') {
        initialized = true;
        calibrationFlag = 1;
  
        userId = strtok(NULL, ",");

        //Check for left or right handed
        int length_string = userId.length();
        if(userId[length_string-1] == 'L'){
          rightState = 0;
        }
        else{
          rightState = 1;
        }
        
        startTime = strtok(NULL, ",");

        //Create file
        createDataFile("CalibrationRun");

        //Write the Unity Time to the data file
        dataFile = SD.open(trialLabelBuffer, FILE_WRITE);
        if (dataFile) {
          dataFile.print("Unity Time:   ");
          dataFile.println(startTime); //1 Byte //AnalogB2
          dataFile.close();
        }
      }
    }
    //a for actively writing; data is read from unity and the microcontroller logs data
    else if (startState[0] == 'a'){
      //Convert setAngle to int
      incomingIdx = strtok(NULL, ",");
      setPoint = atoi(incomingIdx);
      if(setPoint > 135){
        setPoint = 0;
      }
      if(setPoint < -135){
        setPoint = 0;
      }
  
      //Convert performanceGain to int
      incomingIdx = strtok(NULL, ",");
      perfGain = atoi(incomingIdx);
      if(perfGain > 7){
        perfGain = 8;
      }
      if(perfGain < 0){
        perfGain = 0;
      }

      //Convert controlMode to int
      incomingIdx = strtok(NULL, ",");
      controlMode = atoi(incomingIdx);
      if(controlMode > 8){
        controlMode = 0;
      }
      if(controlMode < 0){
        controlMode = 0;
      }
    }
  }
}

void readEncoder() {
  rawEncoder = encoder.read();
  robotAngle = (270.0 / 2875.0) * rawEncoder; //max angle range divided by raw encoder range
}

void readPushbutton() {
  // check if the pushbutton is pressed.
  buttonState = digitalRead(buttonPin);
  //buttonState is HIGH == not pressed:
  if (buttonState == HIGH) {
    gripTime = 0; //resets time counter
  }

  //Debouncing array
  for(int i = 0; i<9; i++){
    buttonArray[i] = buttonArray[i+1];
  }
  buttonArray[9] = buttonState;

  float sum =0.0;

  for(int j = 0; j<10; j++){
    sum = sum + buttonArray[j];
  }

  if(sum/10 == 1){
    button1 = 0;
  }
  else{
    button1 = 1;
  }
  
}

void readHandleSensors() {
  //Read Sensors, may need to be rearranged based on which is plugged in where
  fsrReading1 = analogRead(fsrPin1);
  fsrReading2 = analogRead(fsrPin2);
  fsrReading3 = analogRead(fsrPin3);
  fsrReading4 = analogRead(fsrPin4);

  //Calculate forces in Newtons using log-log fsr calibration curve
  fsr1 = exp(17.25 - 1.73 * log((10000 * (1023 - fsrReading1)) / fsrReading1));
  fsr2 = exp(17.25 - 1.73 * log((10000 * (1023 - fsrReading2)) / fsrReading2));
  fsr3 = exp(17.25 - 1.73 * log((10000 * (1023 - fsrReading3)) / fsrReading3));
  fsr4 = exp(17.25 - 1.73 * log((10000 * (1023 - fsrReading4)) / fsrReading4));

  if(controlMode <=3){
    //Output net force on handle
    net_force = fsr1 + fsr2 - fsr3 - fsr4;
  }
  else if(controlMode == 4 || controlMode == 5){
//    fsr1 += -10.22;
//    fsr2 += -5.59;
//    fsr3 += -1.58;
//    fsr4 += -2.83;
    net_force = fsr1 + fsr2 + 5*(-fsr3 - fsr4);
  }
  else if(controlMode == 6 || controlMode == 7){
    net_force = fsr1 + fsr2 - fsr3 - fsr4;
  }
}

void readBioSensors() {
  heartrate.getValue(heartratePin);
  HR = heartrate.getRate();
  if(HR>=50 && HR <=148)  {
    oldHR = HR;
  }
  
  GSR = analogRead(GSRPin);

  humanResistance = ((1024.0+2*GSR)*10000)/(512.0-GSR);
  microSiemens = 1000000.0/humanResistance;
}

void readIMU(){
  // read raw accel/gyro measurements from device
  shouldergyro.getMotion6(&ax_shoulder, &ay_shoulder, &az_shoulder, &gx_shoulder, &gy_shoulder, &gz_shoulder);
  sternumgyro.getMotion6(&ax_sternum, &ay_sternum, &az_sternum, &gx_sternum, &gy_sternum, &gz_sternum);
}

void setPinchAngle(){
  pot_val=analogRead(pot_pin);
  pinch_angle=-0.265*pot_val+236;
  if (pinch_angle>set_angle) {
    myStepper.step(1);
  }
  if (pinch_angle<set_angle) {
    myStepper.step(-1);
  }
}

//TO DO: FINISH MODE SWITCHING FOR SERVO OUT
void setServoOutput() {

  
  ////////////////////////////////
  //MODE 0 - GRAVITY COMPENSATED//
  ////////////////////////////////
  if (controlMode==0) {
    servoOutput = .5;
    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }
    servoOutput = 0.5 + .5*net_force;
    if (servoOutput > 0.9)
    {
      servoOutput = 0.9;
    }
    if (servoOutput < 0.1)
    {
      servoOutput = 0.1;
    }

    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutput * 256));
  }

  //////////////////////
  //MODE 1 - RESISTIVE//
  //////////////////////

  //////////////////////
  //MODE 2 - ASSISTIVE//
  //////////////////////
  if (controlMode==2) {
    //Map Kp
    if(perfGain>4){
      Kp = 0.004-.001*(perfGain-4);
      Ki = 0.000025;
    }
    else if(perfGain<=4){
      Kp = 0.004+0.001*(4-perfGain);
      Ki = 0.000025;
    }
    else{
      Kp = 0;
      Ki = 0;
    }

    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }

    //Add on a small force control
    float forceAddition = 0.0;
    forceAddition = (0.2*net_force);

    //PID controller
    error = -(setPoint - robotAngle);
    //Serial.println(error);
    total_error += error;
    if (total_error > total_error_max) total_error = total_error_max;
    if (total_error < -total_error_max) total_error = -total_error_max;
    //servoOutput = .5 + (Kp * error);
    servoOutput = .5 + (Kp*error+Ki*total_error + Kd*(error - error_last)); //originally error - error_last
    error_last = error;
    servoOutputFilt = .6*servoOutput + .4*oldServoOutput;
    servoOutputFilt += forceAddition;

    //Cap max/min servo output
    double outMax = .9;
    double outMin = .1;
    if (servoOutputFilt > outMax) servoOutputFilt = outMax;
    if (servoOutputFilt < outMin) servoOutputFilt = outMin; 
    
    oldServoOutput = servoOutput;

    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutputFilt * 256));
  }

  
  ///////////////////////////////
  //MODE 3 - DYNAMIC RESISTANCE//
  ///////////////////////////////
  if (controlMode==3) {
    servoOutput = .5;

    //Map control gain
    if(perfGain>4){
      controlGain = .6-.1*(perfGain-4);
      stepper_speed = 50+30*(perfGain-4);
    }
    else if(perfGain<=4){
      controlGain = .6+.225*(4-perfGain);
      stepper_speed = 50-10*(perfGain-4);
    }
    else{
      controlGain = 0;
      stepper_speed = 30;
    }
    
    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }
    servoOutput = 0.5 + (controlGain*net_force);
    if (servoOutput > 0.9)
    {
      servoOutput = 0.9;
    }
    if (servoOutput < 0.1)
    {
      servoOutput = 0.1;
    }
    
    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutput * 256));
  }
  
  /////////////////////////////////////////////////////
  //MODE 4 - GRAVITY COMPENSATED PRONATION SUPINATION//
  /////////////////////////////////////////////////////
  if (controlMode==4) {
    servoOutput = .5;
    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }
    servoOutput = 0.5 + 1.6*net_force;
    if (servoOutput > 0.9)
    {
      servoOutput = 0.9;
    }
    if (servoOutput < 0.1)
    {
      servoOutput = 0.1;
    }
    if(robotAngle > 75 || robotAngle < -75){
      servoOutput = 0.5;
    }

    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutput * 256));
  }
  
  ///////////////////////////////////////////////////
  //MODE 5 - PRONATION/SUPINATION DYNAMIC RESISTANCE//
  ////////////////////////////////////////////////////
  if (controlMode==5) {
    servoOutput = .5;

    //Map control gain
    if(perfGain>4){
      controlGain = 1.6-.2*(perfGain-4);
    }
    else if(perfGain<=4){
      controlGain = 1.6+.15*(4-perfGain);
    }
    else{
      controlGain = 1;
    }
    
    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }
    servoOutput = 0.5 + (controlGain*net_force);
    if (servoOutput > 0.9)
    {
      servoOutput = 0.9;
    }
    if (servoOutput < 0.1)
    {
      servoOutput = 0.1;
    }
    if(robotAngle > 75 || robotAngle < -75){
      servoOutput = 0.5;
    }
    
    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutput * 256));
  }
  
  //////////////////////////////////////
  //MODE 6 - GRAVITY COMPENSATED PINCH//
  //////////////////////////////////////
  if (controlMode==6) {
    servoOutput = .5;
    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }
    servoOutput = 0.5 + .5*net_force;
    if (servoOutput > 0.9)
    {
      servoOutput = 0.9;
    }
    if (servoOutput < 0.1)
    {
      servoOutput = 0.1;
    }

    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutput * 256));
    myStepper.setSpeed(30);
  }
  
  /////////////////////////////////////
  //MODE 7 - DYNAMIC RESISTANCE PINCH//
  /////////////////////////////////////
  if (controlMode==7) {
    servoOutput = .5;

    //Map control gain
    if(perfGain>4){
      controlGain = .5-.05*(perfGain-4);
      stepper_speed = 50+30*(perfGain-4);
    }
    else if(perfGain<=4){
      controlGain = .5+.05*(4-perfGain);
      stepper_speed = 50-10*(perfGain-4);
    }
    else{
      controlGain = 0;
      stepper_speed = 30;
    }
    
    if (net_force > 0.5)
    {
      net_force = 0.5;
    }
    if (net_force < -0.5)
    {
      net_force = -0.5;
    }
    servoOutput = 0.5 + (controlGain*net_force);
    if (servoOutput > 0.9)
    {
      servoOutput = 0.9;
    }
    if (servoOutput < 0.1)
    {
      servoOutput = 0.1;
    }
    
    //Sends servo control value
    analogWrite(servoOutputPin, round(servoOutput * 256));
    myStepper.setSpeed(stepper_speed); //Changes resistance
  }
  
}
  
void mapSerialOutput() {
  // use this space to manage variable resolution and scale
  theta1 = round(map(robotAngle, -135, 135, 0, 270)); // -135-135 -> 0-270
  force1 = round(net_force * 100); // Newtons

  //Pinch handle
  if(pinch_angle<=90){
    theta2 = pinch_angle;
  }
  else{
    theta2 = map(pinch_angle, 90, 180, 90,0);
  }
  
  GSR1 = round(microSiemens*3);
  HR1 = oldHR-50; //YET TO BE IMPLEMENTED
}

void sendSerialData() {

//   Serial print state as comma separated values
  Serial.print(theta1); //3 bytes
  Serial.print(',');
  Serial.print(force1); //3 bytes
  Serial.print(',');
  Serial.print(button1); //1 Byte
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogX1
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogY1
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogB1
  Serial.print(',');
  Serial.print(GSR1); //2 byte
  Serial.print(',');
  Serial.print(HR1); // 2 byte
  Serial.print(',');

  Serial.print(theta2); //3 bytes //theta2
  Serial.print(',');
  Serial.print(000); //3 bytes //force2
  Serial.print(',');
  Serial.print(0); //1 Byte //button2
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogX2
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogY2
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogB2
  Serial.print(',');
  Serial.print(0); //1 byte //GSR2
  Serial.print(',');
  Serial.print(0); // 1 byte //HR2
  
  Serial.println(); //Carriage return
}

void collectCalibrationData(){
  calibrationFlag = 0;

  //Calculate calibration value
  for(int j = 0; j<60; j++){
    calibrationValueGSR = calibrationValueGSR + siemensArray[j];
    calibrationValueHR = calibrationValueHR + heartrateArray[j];
  }
  calibrationValueGSR = round(calibrationValueGSR/60);
  calibrationValueHR = round(calibrationValueHR/60);

  //Store calibrated value in SD card
  createDataFile("CalibrationGSR");
  dataFile = SD.open(trialLabelBuffer, FILE_WRITE);
  if (dataFile) {
    dataFile.print(calibrationValueGSR);
    dataFile.close();
  }
  createDataFile("CalibrationHR");
  dataFile = SD.open(trialLabelBuffer, FILE_WRITE);
  if (dataFile) {
    dataFile.print(calibrationValueHR);
    dataFile.close();
  }
}

void sendCalibrationData() {

//   Serial print state as comma separated values
  Serial.print(theta1); //3 bytes
  Serial.print(',');
  Serial.print(force1); //3 bytes
  Serial.print(',');
  Serial.print(button1); //1 Byte
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogX1
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogY1
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogB1
  Serial.print(',');
  Serial.print(calibrationValueGSR); //2 byte
  Serial.print(',');
  Serial.print(calibrationValueHR); // 2 byte
  Serial.print(',');

  Serial.print(theta2); //3 bytes //theta2
  Serial.print(',');
  Serial.print(000); //3 bytes //force2
  Serial.print(',');
  Serial.print(0); //1 Byte //button2
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogX2
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogY2
  Serial.print(',');
  Serial.print(0); //1 Byte //AnalogB2
  Serial.print(',');
  Serial.print(0); //1 byte //GSR2
  Serial.print(',');
  Serial.print(0); // 1 byte //HR2
  
  Serial.println(); //Carriage return
}

void updateCalibrationData(){
  String label;
  char labelBuffer[64];

  //Pull GSR calibration value
  label = userId + "/" + "CalibrationGSR" + ".txt";
  label.toCharArray(labelBuffer, label.length() + 1);
  if(SD.exists(labelBuffer)){
    dataFile = SD.open(labelBuffer, FILE_READ);
    if(dataFile){
      String text = dataFile.readStringUntil('\n');
      calibrationValueGSR = text.toInt();
      dataFile.close();
    }
  }
  else{
    calibrationValueGSR = 45;
  }

  //Pull HR calibration value
  label = userId + "/" + "CalibrationHR" + ".txt";
  label.toCharArray(labelBuffer, label.length() + 1);
  if(SD.exists(labelBuffer)){
    dataFile = SD.open(labelBuffer, FILE_READ);
    if(dataFile){
      String text = dataFile.readStringUntil('\n');
      calibrationValueHR = text.toInt();
      dataFile.close();
    }
  }
  else{
    calibrationValueHR = 70;
  }
}

void sendTestData(){
  //Typical printouts
  Serial.print("Encoder:  ");
  Serial.print(robotAngle);
  Serial.print(",   Force:  ");
  Serial.print(force1);
  Serial.print(",   Theta 2:  ");
  Serial.print(theta2);
  Serial.print(",   HR:  ");
  Serial.print(oldHR);
  Serial.print(",   GSR:  ");
  Serial.print(GSR);
  Serial.print(",   Shoulder IMU:  ");
  Serial.print(az_shoulder);
  Serial.print(",   Sternum IMU:  ");
  Serial.print(az_sternum);

  //PID printouts
//  Serial.print(error); 
//  Serial.print(",\t");
//  Serial.print("Total Error: ");
//  Serial.print(total_error); 
//  Serial.print(",\t");
//  Serial.print("Error Last: ");
//  Serial.print(error_last); 
//  Serial.print(",\t");
//  Serial.print("Servo Output: ");
//  Serial.print(servoOutputFilt); 

  //FSR printouts
//  Serial.print("FSR1:  ");
//  Serial.print(fsr1);
//  Serial.print(",   FSR2:  ");
//  Serial.print(fsr2);
//  Serial.print(",   FSR3:  ");
//  Serial.print(fsr3);
//  Serial.print(",   FSR4:  ");
//  Serial.print(fsr4);
//  Serial.print(",   Net Force:  ");
//  Serial.print(net_force);

  
  Serial.println(); //Carriage return
}

void logSampleTime(){
  //Calculate time since last function call
  previousTime = currentTime;
  currentTime = millis();
  sampleTime = currentTime - previousTime;
}

void logSerialData() {
  dataFile = SD.open(trialLabelBuffer, FILE_WRITE);
  if (dataFile) {

    //Unity Data
    dataFile.print(controlMode);
    dataFile.print(",");
    dataFile.print(perfGain);
    dataFile.print(",");
    dataFile.print(setPoint);
    dataFile.print(",");

    //Robot Data
    dataFile.print(robotAngle);
    dataFile.print(",");
    dataFile.print(force1);
    dataFile.print(",");
    dataFile.print(button1);
    dataFile.print(",");
    dataFile.print(0); //AnalogX1
    dataFile.print(",");
    dataFile.print(0); //AnalogY1
    dataFile.print(",");
    dataFile.print(0); //AnalogB1
    dataFile.print(",");
    dataFile.print(microSiemens);
    dataFile.print(",");
    dataFile.print(oldHR);
    dataFile.print(",");

    dataFile.print(pinch_angle); //2 bytes
    dataFile.print(',');
    dataFile.print(000); //3 bytes //force2
    dataFile.print(',');
    dataFile.print(0); //1 Byte //button2
    dataFile.print(',');
    dataFile.print(0); //1 Byte //AnalogX2
    dataFile.print(',');
    dataFile.print(0); //1 Byte //AnalogY2
    dataFile.print(',');
    dataFile.print(0); //1 Byte //AnalogB2
    dataFile.print(',');
    dataFile.print(0); //1 byte //GSR2
    dataFile.print(',');
    dataFile.print(0); // 1 byte //HR2
    dataFile.print(',');

    //Shoulder IMU Data
    dataFile.print(ax_shoulder);
    dataFile.print(',');
    dataFile.print(ay_shoulder);
    dataFile.print(',');
    dataFile.print(az_shoulder);
    dataFile.print(',');
    dataFile.print(gx_shoulder);
    dataFile.print(',');
    dataFile.print(gy_shoulder);
    dataFile.print(',');
    dataFile.print(gz_shoulder);
    dataFile.print(',');

    //Sternum IMU Data
    dataFile.print(ax_sternum);
    dataFile.print(',');
    dataFile.print(ay_sternum);
    dataFile.print(',');
    dataFile.print(az_sternum);
    dataFile.print(',');
    dataFile.print(gx_sternum);
    dataFile.print(',');
    dataFile.print(gy_sternum);
    dataFile.print(',');
    dataFile.print(gz_sternum);
    dataFile.print(',');
    
    dataFile.print(sampleTime); //Time since last sample in ms
    
    dataFile.println();
    dataFile.close();
  }
}



void loop() {
  readSerialInput(); //Bring in new serial data from the PC and assign to variables

  //readSerialInputNew();

  readEncoder(); // Read robot angle

  readPushbutton(); //Read off hand pushbutton

  readHandleSensors(); // Read 4 analog inputs for force handle by default

  readBioSensors(); //Read HR and GSR sensors

  readIMU(); //Read shoulder and sternum IMUs

  setServoOutput(); // Set motor output based on sensors, serial conditions

  setPinchAngle();

  mapSerialOutput(); // scale variables to be efficient for small data packet
  
  if (initialized && (startState[0] == 'a' || startState[0] == 's')) {
    sendSerialData(); // Send data to PC

    logSampleTime(); //Calculate time since last function call

    logSerialData(); //Log the data
  }
  else if (initialized && startState[0] == 'c') {
    sendSerialData(); // Send data to PC

    logSampleTime(); //Calculate time since last function call

    logSerialData(); //Log the data

    //Increment calibration array
    for(int i = 0; i<59; i++){
      siemensArray[i] = siemensArray[i+1];
      heartrateArray[i] = heartrateArray[i+1];
    }
    siemensArray[59] = GSR1;
    heartrateArray[59] = HR1;
  }
  else if (startState[0] == 'r'){

    logSampleTime(); //Calculate time since last function call

    logSerialData(); //Log the data

    //Pull calibration data from txt file if it exists
    updateCalibrationData();

    //Send the calibration data to Unity
    sendCalibrationData();
  }
  else{
    if(calibrationFlag == 1){
      collectCalibrationData();
    }

    Serial.print("waiting for unity prompt\t");
    sendTestData();
  }

  delay(12);

}

/*
void readSerialInputNew() {

  static boolean recvInProgress = false;
  char rc;
  bool newData = false;
  String receivedChars = "";
  int a = 0;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != '>') {
        receivedChars += rc;
      }
      else {
        recvInProgress = false;
        newData = true;
      }
    }

    else if (rc == 's' || rc == 'a') {
      recvInProgress = true;
      if (rc == 's') {
        a = 1;
      }
      if (rc == 'a') {
        a = 2;
      }
    }
  }

  if (newData) {
    if (a == 1) {
      userId = "";
      startTime = "";
      int secondCommaIndex = 0;
      for (int i = 2; i < receivedChars.length(); i++) {
        if (receivedChars[i] != ',') {
          if (secondCommaIndex == 0) {
            userId += receivedChars[i];
          }
          else {
            startTime += receivedChars[i];
          }
        }
        else {
          secondCommaIndex = i;
        }
      }
      Serial.println(userId);
      
      if(!useridset){
        useridset = true;
        gameStarted();
      }
    }
    else if (a == 2 && useridset) {
      int commas = 0;
      String setPointAsString = "";
      String performanceAsString = "";
      String modeAsString = "";
      for (int i = 2; i < receivedChars.length(); i++) {
        if (receivedChars[i] != ',') {
          if (commas == 0) {
            setPointAsString += receivedChars[i];
          }
          else if (commas == 1) {
            performanceAsString += receivedChars[i];
          }
          else {
            modeAsString += receivedChars[i];
          }
        }
        else {
          commas++;
        }
      }

      setPoint = setPointAsString.toInt();
      perfGain = performanceAsString.toInt();
      controlMode = modeAsString.toInt();
    }

    if (receivedChars == "agameover") {
      gameEnded();
    }
  }
}

void gameStarted() {
  //Create File
  createDataFile();
}

void gameEnded() {
  useridset = false;
} */
