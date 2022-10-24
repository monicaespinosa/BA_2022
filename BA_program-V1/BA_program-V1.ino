#include <Wire.h>

// Output pins Arduino to actuators
const int LEDStrip = 3;
const int PowerRes = 9;           //Temperature modifier
const int InnerVent = 10;
const int OuterVents = 11;

// Input pins to sensors to Arduino
const int LEDPot = 19;
//SDA and SCL pins (D18 and D19) connect to Thermal sensors
//All sensors work, right now I'll be using #0, #1, #2, and #3

const int numSens = 4;
int resBits = 3;
int conSens;
// I2C Addresses for the sensors
int I2CAddress_Sensors [numSens] = {0b1001000,
                                    0b1001001,
                                    0b1001010,
                                    0b1001011};

// Registers of the sensor accessible by the programmer
int RegTA = 0x00;
int RegConfig = 0x01;
int RegTHyst = 0x10;
int RegTSet = 0x11;

float sensTemp[4];
float meanTemp;
float dutyCycleIn;
float dutyCycleOut;
bool heat;
const float desiredTemp = 27.5;
const float rangeTemp = 1.25;

//interval related settings (avoid using dealy() function)
const long intervalSensors = 300;        //On datasheet says that refresh time for max resolution is 240ms
unsigned long previousMillis = 0;
const long intervalTempCheck = 500;
unsigned long previousCheck = 0;

void setup() {
  //Join I2C bus
  Wire.begin();
  //debugging
  Serial.begin(9600);

  pinMode(LEDStrip, OUTPUT);
  pinMode(PowerRes, OUTPUT);
  pinMode(InnerVent, OUTPUT);
  pinMode(OuterVents, OUTPUT);

  pinMode(LEDPot, INPUT);

  //configuration Temperature sensors
  for (int i = 0; i < numSens; i++) {
    Wire.beginTransmission(I2CAddress_Sensors[i]);
    Wire.write(RegConfig);
    //Setting the resolution to 3 Bits
    Wire.write(0b01000000);
    Wire.endTransmission();

    //Set the sensors to send the data of environment temperature
    Wire.beginTransmission(I2CAddress_Sensors[i]);  //Open transmission to device according to address
    Wire.write(RegTA);                              //Set register Point - Register to ambient temperature
    Wire.endTransmission();
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= intervalSensors) {
    previousMillis = currentMillis;
    conSens = 0;
    for (int i = 0; i < numSens; i++) {
      Wire.requestFrom(I2CAddress_Sensors[i], 2);     //Requests 2 bytes from the slave, due to the nature of the reading
      
      while (Wire.available()){
        int reg1 = Wire.read();                       //The first byte corresponds to the integer part of the lecture
        int reg2 = Wire.read();                       //The second byte is the resolution chosen by the programer

        int tempInt = reg1 << resBits;
        int tempDec = reg2 >> 4;

        int tempData = tempInt | tempDec;
        sensTemp[i] = tempData * pow(2, -resBits);
        
        conSens++;
        
        /*
        //debugging
        Serial.print("Temp Sensor #");
        Serial.print(conSens);
        Serial.print(": ");
        Serial.println(sensTemp[i]);
        */
      }
    }
  }
  if(conSens =! 0){
    meanTemp = 0;
    for(int i = 1; i <= conSens; i++){
      meanTemp += sensTemp[i];
    }
    meanTemp = meanTemp / conSens;
    
    /*
    //debugging
    Serial.print("mean temperature: ");
    Serial.println(meanTemp);
    */
  }
  //Temperature control, its more like an ON/OFF mixed with a basic PWM - the values 
  //of the PWM wher whoosen arbitrarily
  if (currentMillis - previousCheck >= intervalTempCheck) {
    previousCheck = currentMillis;
    if(((meanTemp == desiredTemp+rangeTemp)||(meanTemp == desiredTemp-rangeTemp)||
        (meanTemp < desiredTemp+rangeTemp)&&(meanTemp > desiredTemp-rangeTemp))){
      heat = LOW;
      dutyCycleIn = 0.82;
      dutyCycleOut = 0.90;
    }else if(meanTemp < desiredTemp - rangeTemp ){
      heat = HIGH;
      dutyCycleIn = 0.85;
      dutyCycleOut = 0.95;
    }else if(meanTemp > desiredTemp+rangeTemp){
      heat = LOW;
      dutyCycleIn = 1;
      dutyCycleOut = 1;
    }
    digitalWrite(PowerRes, heat);
    analogWrite(InnerVent, 255*dutyCycleIn);
    analogWrite(OuterVents, 255*dutyCycleOut);
  }
  
}
