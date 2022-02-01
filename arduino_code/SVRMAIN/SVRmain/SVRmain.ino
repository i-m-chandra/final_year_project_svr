#include <Wire.h>
#include <VirtualWire.h>
char *data;
String readString = "";
int sent=0;
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
const int sensorPinTHUMB = A0;
const int sensorPinINDEX = A1;
const int sensorPinMIDDLE = A2;
const int sensorPinRING = A3;
const int sensorPinPINKEY = A6;
const int XSCL = A5;
const int XSDA = A4;
const int Vcc1 = 2;
const int Vcc2 = 13;

// variables:
int sensorValueTHUMB = 0;
int sensorValueINDEX = 0;
int sensorValueMIDDLE = 0;
int sensorValueRING = 0;
int sensorValuePINKEY = 0;

int sensorMinTHUMB = 1023;
int sensorMinINDEX = 1023;
int sensorMinMIDDLE = 1023;
int sensorMinRING = 1023;
int sensorMinPINKEY = 1023;

int sensorMaxTHUMB = 0;
int sensorMaxINDEX = 0;
int sensorMaxMIDDLE = 0;
int sensorMaxRING = 0;
int sensorMaxPINKEY = 0;

float palmAngle = 0.0,g;

//Contact Sensor Pins
int t = A7;
int i= 4;
int m= 5;
int r= 6;
int p= 7;
//Contact Sensor Data
int T = 0;
int I = 0;
int M = 0;
int R = 0;
int P = 0;

void setup() {
    vw_set_ptt_inverted(true);
  vw_set_tx_pin(12);
  vw_setup(4000);
  Serial.begin(9600);
  Wire.begin();
  //Serial.print ("1");

  pinMode(i, INPUT);
  pinMode(m, INPUT);
  pinMode(r, INPUT);
  pinMode(p, INPUT);

  pinMode(Vcc1,OUTPUT);
  pinMode(Vcc2,OUTPUT);
  digitalWrite(Vcc1,HIGH);
  digitalWrite(Vcc2,HIGH);
  
  setupMPU();

  // calibrate during the first five seconds 
  while (millis() < 5000) {
      sensorValueTHUMB = analogRead(sensorPinTHUMB);
      sensorValueINDEX = analogRead(sensorPinINDEX);
      sensorValueMIDDLE = analogRead(sensorPinMIDDLE);
      sensorValueRING = analogRead(sensorPinRING);
      sensorValuePINKEY = analogRead(sensorPinPINKEY);
//--------------------------------------------------
    // record the maximum sensor value
    if (sensorValueTHUMB > sensorMaxTHUMB) {
      sensorMaxTHUMB = sensorValueTHUMB;
    }
       // record the maximum sensor value
    if (sensorValueINDEX > sensorMaxINDEX) {
      sensorMaxINDEX = sensorValueINDEX;
    }
   // record the maximum sensor value
    if (sensorValueMIDDLE > sensorMaxMIDDLE) {
      sensorMaxMIDDLE = sensorValueMIDDLE;
    }
    // record the maximum sensor value
    if (sensorValueRING > sensorMaxRING) {
      sensorMaxRING = sensorValueRING;
    }
    // record the minimum sensor value
    if (sensorValueRING < sensorMinRING) {
      sensorMinRING = sensorValueRING;
    }
     // record the maximum sensor value
    if (sensorValuePINKEY > sensorMaxPINKEY) {
      sensorMaxPINKEY = sensorValuePINKEY;
    }
    //--------------------------------------------------------

    // record the minimum sensor value
    if (sensorValueTHUMB < sensorMinTHUMB) {
      sensorMinTHUMB = sensorValueTHUMB;
    }
    
       // record the minimum sensor value
    if (sensorValueINDEX < sensorMinINDEX) {
      sensorMinINDEX = sensorValueINDEX;
    }
        // record the minimum sensor value
    if (sensorValueMIDDLE < sensorMinMIDDLE) {
      sensorMinMIDDLE = sensorValueMIDDLE;
    }
    // record the minimum sensor value
    if (sensorValueRING < sensorMinRING) {
      sensorMinRING = sensorValueRING;
    }
    // record the minimum sensor value
    if (sensorValuePINKEY < sensorMinPINKEY) {
      sensorMinPINKEY = sensorValuePINKEY;
    }
  }

  // signal the end of the calibration period
 

  //Serial.print ("4");
  Serial.print ("LowTHUMB = ");
  Serial.println (sensorMinTHUMB);
  Serial.print ("HighTHUMB = ");
  Serial.println (sensorMaxTHUMB);
  
  Serial.println ("             ");
  
  Serial.print ("LowINDEX = ");
  Serial.println (sensorMinINDEX);
  Serial.print ("HighINDEX = ");
  Serial.println (sensorMaxINDEX);
  
  Serial.println ("             "); 
  
  Serial.print ("LowMIDDLE = ");
  Serial.println (sensorMinMIDDLE);
  Serial.print ("HighMIDDLE = ");
  Serial.println (sensorMaxMIDDLE);
  
  Serial.println ("             "); 
  
   Serial.print ("LowRING = ");
  Serial.println (sensorMinRING);
  Serial.print ("HighRING = ");
  Serial.println (sensorMaxRING);
  
  Serial.println ("             "); 
  
  Serial.print ("LowPINKEY = ");
  Serial.println (sensorMinPINKEY);
  Serial.print ("HighPINKEY = ");
  Serial.println (sensorMaxPINKEY);
  
  Serial.println ("             ");

  //Serial.print ("5");
  delay (1000);
}


void loop() {
  T = analogRead(t);
  I = digitalRead(i);
  M= digitalRead(m);
  R= digitalRead(r);
  P= digitalRead(p);
    while (Serial.available()){
      sent = sent + 1;
      break;
    }
    if(sent==1){
      sendData();
    }
 
  //Serial.println("6");
  recordAccelRegisters();
  //Serial.println("7");
  fingures();
  //Serial.print ("8");
  printData();
  //Serial.print ("9");
  delay(1000);
}


void sendData(){
    while(!Serial.available()) {}
  // serial read section
    while (Serial.available())
  {
    if (Serial.available() >0)
    {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    }
  }

  if (readString.length() >0)
  {
    Serial.print("Arduino received: ");  
    Serial.println(readString); //see what was received
  }
  delay(500);

  Serial.flush(); 
  if (readString == "5"){
  data="1";
  vw_send((uint8_t *)data, strlen(data));
  vw_wait_tx();
  //Serial.println(data);
  //digitalWrite(ledPin,HIGH);
  }
     else if (readString == "6"){
  data="2";
  vw_send((uint8_t *)data, strlen(data));
  vw_wait_tx();
  Serial.print(data);
  //digitalWrite(ledPin,HIGH);
  }
     else if (readString == "7"){
  data="3";
  vw_send((uint8_t *)data, strlen(data));
  vw_wait_tx();
  Serial.print(data);
  //digitalWrite(ledPin,HIGH);
  }
     else if (readString == "8"){
  data="4";
  vw_send((uint8_t *)data, strlen(data));
  vw_wait_tx();
  Serial.print(data);
  //digitalWrite(ledPin,HIGH);
  }
      else if (readString == "9"){
  data="5";
  vw_send((uint8_t *)data, strlen(data));
  vw_wait_tx();
  Serial.print(data);
  //digitalWrite(ledPin,HIGH);
  }
  readString="";
  sent = 0;
  delay(1000);
  Serial.flush();

 
 // data="0";
 // vw_send((uint8_t *)data, strlen(data));
  //vw_wait_tx();
 // digitalWrite(ledPin,LOW);
// Serial.println(data);
 // delay(2000);



}
void setupMPU(){
  
  //Serial.print ("2");
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  
  //Serial.print ("3");
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  
  gForceX = (float)accelX / 16384.0;
  gForceY = (float)accelY / 16384.0; 
  gForceZ = (float)accelZ / 16384.0;
  g = (float)sqrt(sq(gForceX)+sq(gForceY)+sq(gForceZ));
  palmAngle = (float)(asin(gForceX/g)*(180.0/3.14));
   
}

void fingures(){

  // read the sensor:
  sensorValueTHUMB = analogRead(sensorPinTHUMB);
  sensorValueINDEX = analogRead(sensorPinINDEX);
  sensorValueMIDDLE = analogRead(sensorPinMIDDLE);
  sensorValueRING = analogRead(sensorPinRING);
  sensorValuePINKEY = analogRead(sensorPinPINKEY);

  // apply the calibration to the sensor reading
  sensorValueTHUMB = map(sensorValueTHUMB, sensorMinTHUMB, sensorMaxTHUMB, 1, 255);
  sensorValueINDEX = map(sensorValueINDEX, sensorMinINDEX, sensorMaxINDEX, 1, 255);
  sensorValueMIDDLE = map(sensorValueMIDDLE, sensorMinMIDDLE, sensorMaxMIDDLE, 1, 255);
  sensorValueRING = map(sensorValueRING, sensorMinRING, sensorMaxRING, 1, 255);
  sensorValuePINKEY = map(sensorValuePINKEY, sensorMinPINKEY, sensorMaxPINKEY, 1, 255);

  // in case the sensor value is outside the range seen during calibration
  sensorValueTHUMB = constrain(sensorValueTHUMB, 1, 255);
  sensorValueINDEX = constrain(sensorValueINDEX, 1, 255);
  sensorValueMIDDLE = constrain(sensorValueMIDDLE, 1, 255);
  sensorValueRING = constrain(sensorValueRING, 1, 255);
  sensorValuePINKEY = constrain(sensorValuePINKEY, 1, 255);
}

void printData() {
  Serial.print(gForceX);
  Serial.print(",");
  Serial.print(gForceY);
  Serial.print(",");
  Serial.print(gForceZ);
  Serial.print(",");
  
  Serial.print (sensorValueTHUMB);
  Serial.print(",");
  Serial.print (sensorValueINDEX);
  Serial.print(",");
  Serial.print (sensorValueMIDDLE);
  Serial.print(",");
  Serial.print (sensorValueRING);
  Serial.print(",");
  Serial.print (sensorValuePINKEY);
  Serial.print(",");
  
  Serial.print(T);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(M);
  Serial.print(",");
  Serial.print(R);
  Serial.print(",");
  Serial.print(P);
  Serial.print(",");
  Serial.print(P);
  Serial.println();
}
