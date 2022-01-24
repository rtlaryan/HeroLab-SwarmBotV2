#ifndef SWARMBOT_H
#define SWARMBOT_H
    #include <Arduino.H>
    #include "Adafruit_TinyUSB.h" 
    #include <Wire.h>
    #include <string>

   
    class SwarmBotV2{


        struct Pins
        {
        //Left Encoder I/O 
          byte leftEncoderC1; //"GREEN" Encoder Output 1
          byte leftEncoderC2; //"YELLOW" Encoder Output 2
          byte leftEncoderVCC;//"BLACK" Encoder Power
          byte leftEncoderGND;//"BLUE" Encoder Ground

        //Right Encoder I/O
          byte rightEncoderC1; //"GREEN" Encoder Output 1
          byte rightEncoderC2; //"YELLOW" Encoder Output 2
          byte rightEncoderVCC;//"BLACK" Encoder Power
          byte rightEncoderGND;//"BLUE" Encoder Ground

        //Motor Driver Inputs
          byte leftMotorM1; //"WHITE" Motor Ground
          byte leftMotorM2; //"RED" Motor Positive
          byte driverMode; // Motor Driver Mode 
          byte rightMotorM1; //"WHITE" Motor Ground
          byte rightMotorM2; //"RED" Motor Positi
        }IO;

        int leftMotorFF;
        int rightMotorFF;

        float leftMotorLastSpeed;
        float rightMotorLastSpeed;

      
      //Pysical Constants
        int encoderCPP; //Encoder Ticks per Revolution - Function of Gear Ratio
        int gearRatio; //Motor Gear Ratio
        float wheelDiameter; //Diameter of wheels
        float wheelBase; //Distance betweeen center of wheels
        float ticksPerMeter; //Encoder ticks per 1 linear meter of travel
        double toDegrees = 180/PI; //Convert radian vale to degrees
        double toRadians = PI/180; //Convert degree value to radians

      //Private ODOM Variables
        volatile long leftEncoderValue; //Current Encoder Ticks
        long leftLastEncoded; //Last Encoder Output Value
        long leftLastEncoderValue; //Last Encoder Ticks
        int leftDeltaTicks; //Change in Ticks since last cycle
        
        volatile long rightEncoderValue; //Current Encoder Ticks
        long rightLastEncoded; //Last Encoder Output Value
        long rightLastEncoderValue; //Last Encoder Ticks
        int rightDeltaTicks; //Change in Ticks since last cycle

        double lastTime; //Time of last cycle
        double deltaTime; //Elasped time since last cycle

        float lastX; //X position at last cycle
        float lastY; //Y position at last cycle
        float lastHeading; //Heading at last cycle
 
        float targetX; //Target X position
        float targetY; //Target Y position
        float targetHeading; //Target Heading

        struct Odometry{
          float X = 0;
          float Y = 0;
          float Heading = 0;
          float AngVel = 0;
          float LinVel = 0;
          float leftRPM = 0;
          float rightRPM = 0;
        }Odom;
      



        public:
          SwarmBotV2(std::string name); //Swarmbot Constructor Method

          //Structure for most current Odometry outputs  
          struct PIDController{
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Derivative;
            float LastError;
            float threshold;
            float* valuePointer; 
            float PID(float setPoint);
            void initialize(float P, float I, float D, float* target, float Th = 0);
          }angularVelController, linearVelController, headingController;

          Pins getIO(){
            return IO;
          }          
          
          Odometry getOdom(){
            return Odom;
          }

          void initializePorts();  //Swarmbot Setup Method
          void updateLeftEncoder();
          void updateRightEncoder();
          void updateOdometery();
          void printOdometry(bool velMode, float a, float b);

          void setVelocityPIDSetpoint(float linearVelocity, float angularVelocity); //Swarmbot velocity setpoint method
          void setLeftMotorSpeed(int speed);
          void setRightMotorSpeed(int speed);
          void setMotorSpeed(int leftMotorSpeed, int rightMotorSpeed);
          void moveToPoint(float targetX, float targetY);
          void turnToPoint(float targetX, float targetY);
          
          float angleWrap(float angle, bool isRad);

          void ctrlVelocity(float linear, float angular);

    };



      

#endif