#include <SwarmbotV2.h>
SwarmBotV2 Tim("Tim");

void leftInterrupt(){
  Tim.updateLeftEncoder();
}

void rightInterrupt(){
  Tim.updateRightEncoder();
}


///I2CLINK
bool serialFlag = false;
bool driveMode = true;
float targetX, targetY, linearVelocity, angularVelocity;
float inputArray[3];

union BytesToFloat {
  byte valueBuffer[12];
  float valueReading[3];    
} converter;

void printInfo(){
  for(uint8_t index = 0; index<3; index++){
    Serial.print("The number is: ");
    float data = converter.valueReading[index];
    Serial.println(data);
    inputArray[index] = data;
  }

  serialFlag = false;
}



void receiveEvent(int byteCount){
  for(uint8_t index = 0; index<byteCount; index++){
    converter.valueBuffer[index] = Wire.read();
  }
  serialFlag = true;
}


void sendEvent(){
  float odometryData[5];
  odometryData[0] = Tim.getOdom().X;
  odometryData[1] = Tim.getOdom().Y;
  odometryData[2] = Tim.getOdom().Heading * RAD_TO_DEG;
  odometryData[3] = Tim.getOdom().LinVel;
  odometryData[4] = Tim.getOdom().AngVel;
  Serial.print("Get X: ");
  Serial.println(odometryData[0]);
  Wire.write((byte*)odometryData, sizeof(odometryData));
}

void unpackData(float array[3]){
  linearVelocity = array[0];
  angularVelocity = array[2];

  Tim.setVelocityPIDSetpoint(linearVelocity, angularVelocity);
}

///



void setup() {
  //Initiate i2c bus
  Wire.begin(0x8); 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendEvent);
  
  //Initialize IO
  Tim.initializePorts();

  //Attach Interrupt to Encoder Outputs
  attachInterrupt(Tim.getIO().leftEncoderC1, leftInterrupt, CHANGE);
  attachInterrupt(Tim.getIO().leftEncoderC2, leftInterrupt, CHANGE);

  attachInterrupt(Tim.getIO().rightEncoderC1, rightInterrupt, CHANGE);
  attachInterrupt(Tim.getIO().rightEncoderC2, rightInterrupt, CHANGE);

  
  Serial.begin(9600);
  delay(1000);
}

void loop() {

  //Run Odometry Functions and Print Output
  Tim.updateOdometery();
  Tim.printOdometry(true, linearVelocity, angularVelocity);
  
  //If new message from I2C Bus, Unpack Data
  if(serialFlag){
    printInfo();
    serialFlag = false;
    Serial.println("Recieved: ");
    unpackData(inputArray);
  }

  //Run PID Controller for Linear and Angular Velocity
  Tim.ctrlVelocity(linearVelocity, angularVelocity);
  delay(100);
}