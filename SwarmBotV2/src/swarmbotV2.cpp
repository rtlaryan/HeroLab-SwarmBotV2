#include <SwarmbotV2.h>

//SWARMBOT CLASS METHODS
////SWARMBOT CONSTRUCTOR
SwarmBotV2::SwarmBotV2(std::string name){
    
    //Define Per-bot Ports and PID Constants. 
    ////Can Add more here to define different bots.
    if(name == "Tim"){ 
      IO.leftEncoderC1 = A2;
      IO.leftEncoderC2 = A1;
      IO.leftEncoderVCC = A3;
      IO.leftEncoderGND = A0;

      IO.rightEncoderC1 = 5;
      IO.rightEncoderC2 = 6;
      IO.rightEncoderVCC = A4;
      IO.rightEncoderGND = A5;

      IO.leftMotorM1 = 13;
      IO.leftMotorM2 = 12;
      IO.driverMode = 9;
      IO.rightMotorM1 = 11;
      IO.rightMotorM2 = 10;

      leftMotorFF = 155;
      rightMotorFF = 155;

      gearRatio = 100;
      wheelBase = 0.066;

      angularVelController.initialize(0.5,0.00,0, &Odom.AngVel);
      linearVelController.initialize(0.6,0,0, &Odom.LinVel);
      headingController.initialize(1,0,0, &Odom.Heading);
      Serial.println("Hi, I'm Tim!");
    }
      
    //Initialize Initial value for Odometery and Motor Control
    encoderCPP = 28;
    wheelDiameter = 0.0325;
    ticksPerMeter = (wheelDiameter*PI)/(encoderCPP*gearRatio);

    leftEncoderValue, leftLastEncoded, leftLastEncoderValue, leftDeltaTicks = 0;
    rightEncoderValue, rightLastEncoded, rightLastEncoderValue, rightDeltaTicks = 0;

    lastTime, deltaTime = 0;

    lastX, lastY, lastHeading, targetX, targetY, targetHeading = 0;

    leftMotorLastSpeed, rightMotorLastSpeed = 0;
}


////INITIALIZEPORTS - Setupt Pin I/O
void SwarmBotV2::initializePorts(){
  digitalWrite(IO.leftEncoderVCC, HIGH);
  digitalWrite(IO.leftEncoderGND, LOW);
  pinMode(IO.leftEncoderGND, OUTPUT);
  pinMode(IO.leftEncoderVCC, OUTPUT);

  pinMode(IO.leftEncoderC1, INPUT_PULLUP);
  pinMode(IO.leftEncoderC2, INPUT_PULLUP);
  digitalWrite(IO.leftEncoderC1, HIGH);
  digitalWrite(IO.leftEncoderC2, HIGH);

  digitalWrite(IO.rightEncoderVCC, HIGH);
  digitalWrite(IO.rightEncoderGND, LOW);
  pinMode(IO.rightEncoderGND, OUTPUT);
  pinMode(IO.rightEncoderVCC, OUTPUT);

  pinMode(IO.rightEncoderC1, INPUT_PULLUP);
  pinMode(IO.rightEncoderC2, INPUT_PULLUP);
  digitalWrite(IO.rightEncoderC1, HIGH);
  digitalWrite(IO.rightEncoderC2, HIGH);

  digitalWrite(IO.driverMode, LOW); //Toggle Drive Mode on Pololu Driver Board 
}

////UpdateLeftEncoder - Get Current Encoder Output
void SwarmBotV2::updateLeftEncoder(){

  int MSB = digitalRead(IO.leftEncoderC1); //MSB = most significant bit
  int LSB = digitalRead(IO.leftEncoderC2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB;             //converting the 2 pin value to single number
  int sum = (leftLastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
      leftEncoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
      leftEncoderValue++;

  leftLastEncoded = encoded; //store this value for next time
  }

////UpdateLeftEncoder - Get Current Encoder Output
void SwarmBotV2::updateRightEncoder()
{
  int MSB = digitalRead(IO.rightEncoderC1); //MSB = most significant bit
  int LSB = digitalRead(IO.rightEncoderC2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB;              //converting the 2 pin value to single number
  int sum = (rightLastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
      rightEncoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
      rightEncoderValue++;

  rightLastEncoded = encoded; //store this value for next time
}

////UpdateOdometery - Run Odometry Algorithem
void SwarmBotV2::updateOdometery(){
  //Get Current Program Time
  long currentTime = millis();
  if (currentTime < lastTime) currentTime = lastTime;

  double deltaTimeMSec = currentTime - lastTime;
  double deltaTimeSec = deltaTimeMSec / 1000;
  double deltaTimeMin = deltaTimeMSec / 60000;


  //Get Change in Encoder Reading
  leftDeltaTicks = leftEncoderValue - leftLastEncoderValue;
  rightDeltaTicks = rightEncoderValue - rightLastEncoderValue;
  
  //Calculate Wheel RPM
  Odom.leftRPM = (leftDeltaTicks / (encoderCPP * gearRatio)) / deltaTimeMin;
  Odom.rightRPM = (rightDeltaTicks / (encoderCPP * gearRatio)) / deltaTimeMin;

  //Calculate change in left, right, center of wheelbase, and angle 
  double dLeft = leftDeltaTicks * ticksPerMeter;
  double dRight = rightDeltaTicks * ticksPerMeter;
  double dCenter = (dLeft+dRight)/2;
  double dPhi = (dRight-dLeft)/wheelBase;

  //Update Global Position
  Odom.Heading += dPhi;
 // Odom.Heading = angleWrap(Odom.Heading, true);
  Odom.X += dCenter * cos(Odom.Heading);
  Odom.Y += dCenter * sin(Odom.Heading);

  //Calculate Local Velocity
  double vLeft = dLeft / deltaTimeSec;
  double vRight = dRight / deltaTimeSec;

  double dX = Odom.X - lastX;
  double dY = Odom.Y - lastY;
  
  double vX = dX / deltaTimeSec;
  double vY = dY / deltaTimeSec;

  Odom.LinVel = (vX + vY) / 2;
  Odom.AngVel = (dPhi / deltaTimeSec);

  //Save Current X and Y position
  lastX = Odom.X;
  lastY = Odom.Y;

  //Save Current Encoder Reading
  leftLastEncoderValue = leftEncoderValue;
  rightLastEncoderValue = rightEncoderValue;

  //Save Current Program Time
  lastTime = currentTime;
  
}

//printOdom - Print ControlMode-Relavent Odometry
void SwarmBotV2::printOdometry(bool velMode, float a, float b){
  //Point Based Odometry
  if(velMode == false){
    Serial.print("Current X: ");
    Serial.print(Odom.X, 4);
    Serial.print(" Current Y: ");
    Serial.println(Odom.Y, 4);
    Serial.print("Target X: ");
    Serial.print(a, 4);
    Serial.print(" Target Y: ");
    Serial.println(b, 4);
    Serial.print("Theta: ");
    Serial.println(Odom.Heading * toDegrees);
  }
  //Velocity Based Odometry
  else{
    Serial.print("linVel: ");
    Serial.print(Odom.LinVel, 4);
    Serial.print(" anglarVel: ");
    Serial.println(Odom.AngVel);
    Serial.print("targetLin: ");
    Serial.print(a, 4);
    Serial.print(" targetAng: ");
    Serial.println(b, 4);
  }
  Serial.println();
  Serial.println();
  
}

//setVelocityPIDSetpoint - "Guess" starting point for PID controller
void SwarmBotV2::setVelocityPIDSetpoint(float linear, float angular){
    rightMotorLastSpeed = (linear + (0.5 * angular * wheelBase)) * 100 / 0.29; //Map Target Linear and Angular Speed to Motor Output using Characterization Data
    leftMotorLastSpeed = (linear - (0.5 * angular * wheelBase)) * 100 / 0.29;
}



//setLeftMotorSpeed - Set PWM output for Left Motor
void SwarmBotV2::setLeftMotorSpeed(int speed){
  float outputLeft = abs(speed) + leftMotorFF;
  
  if (speed > 0){
    analogWrite(IO.leftMotorM1, 0);
    analogWrite(IO.leftMotorM2, outputLeft);
  }

  else if (speed < 0){
    analogWrite(IO.leftMotorM1, outputLeft);
    analogWrite(IO.leftMotorM2, 0);
  }

  else{
    analogWrite(IO.leftMotorM1, 0);
    analogWrite(IO.leftMotorM2, 0);
  }
}

//setRightMotorSpeed - Set PWM output for Right Motor
void SwarmBotV2::setRightMotorSpeed(int speed){
  float outputRight = abs(speed) + rightMotorFF;
  
  if (speed > 0){
    analogWrite(IO.rightMotorM1, 0);
    analogWrite(IO.rightMotorM2, outputRight);
  }

  else if (speed < 0){
    analogWrite(IO.rightMotorM1, outputRight);
    analogWrite(IO.rightMotorM2, 0);
  }
  
  else{
    analogWrite(IO.rightMotorM1, 0);
    analogWrite(IO.rightMotorM2, 0);
  }
}

//setMotorSpeed - set speeds for left and right motors
void SwarmBotV2::setMotorSpeed(int leftMotorSpeed, int rightMotorSpeed){
  setLeftMotorSpeed(leftMotorSpeed);
  setRightMotorSpeed(rightMotorSpeed);
}

//AngleWrap - Constrain Angle to +/- 180deg or PIrad
float SwarmBotV2::angleWrap(float angle, bool isRad){
  double coefficient = (PI * isRad) + (180 * !isRad);

  while (abs(angle) > coefficient){
    if (angle < (-1 * coefficient)) angle += 2.0 * coefficient;
    else if (angle > coefficient) angle -= 2.0 * coefficient;
  }

  return angle;
}

//moveToPoint - Global Position Controller
void SwarmBotV2::moveToPoint(float targetX, float targetY){
  float deltaX = targetX - Odom.X;
  float deltaY = targetY - Odom.Y;

  float distance = hypot(deltaX, deltaY);
  float absAngle = atan2(deltaY, deltaX);
  float relAngle = angleWrap(absAngle, true);
  

  float distanceOutput;
  float angleOutput = headingController.PID(relAngle);

  if (distance > 0.02){
    distanceOutput = 20;
  }

  else{
    distanceOutput = 0;
  }

  float leftOut = distanceOutput - angleOutput;
  float rightOut = distanceOutput + angleOutput;
}

//turnToPoint - Turn To Face Global Target
void SwarmBotV2::turnToPoint(float targetX, float targetY){
  float deltaX = targetX - Odom.X;
  float deltaY = targetY - Odom.Y;

  float distance = hypot(deltaX, deltaY);
  float absAngle = atan2(deltaY, deltaX);
  float relAngle = angleWrap(absAngle, true);

  float angleOutput = headingController.PID(relAngle);
  float leftOut = -angleOutput;
  float rightOut = angleOutput;
  setMotorSpeed(leftOut, rightOut);
}

//ctrlVelocity - Maintain Desired Velocity using PID
void SwarmBotV2::ctrlVelocity(float linearTarget, float angularTarget){
  float leftMotorOut = leftMotorLastSpeed + linearVelController.PID(linearTarget) - angularVelController.PID(angularTarget);
  float rightMotorOut = rightMotorLastSpeed + linearVelController.PID(linearTarget) + angularVelController.PID(angularTarget);
  
  leftMotorLastSpeed = leftMotorOut;
  rightMotorLastSpeed = rightMotorOut;
  
  setMotorSpeed(leftMotorOut, rightMotorOut);
}




/////////////PID SUB-CLASS METHODS
void SwarmBotV2::PIDController::initialize(float P, float I, float D, float* value, float Th){
  Kp = P;
  Ki = I;
  Kd = D;
  threshold = Th;
  valuePointer = value;
}

float SwarmBotV2::PIDController::PID(float setPoint){
  float error = setPoint - *valuePointer;
  if (abs(error) < threshold) return 0;
  Integral += error;
  Derivative = error - LastError;
  LastError = error;
  return (Kp*error + Ki*Integral + Kd*Derivative);
}

