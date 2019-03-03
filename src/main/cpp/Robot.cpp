/*
  2019 - Axon

  "🅱️"
    - The Team 5458 Programming Team
*/

#include <string>
#include <Robot.h>
#include <sstream>
#include <WPILib.h>
#include <iostream>
#include <Encoder.h>
#include <frc/Timer.h>
#include <TimedRobot.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/SpeedControllerGroup.h>
#include <NetworkTables/NetworkTable.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>


// Declarations

// PDP
frc::PowerDistributionPanel pdp{5};

// Right Side Motors
WPI_VictorSPX RightMotorOne{6};
WPI_TalonSRX RightMotorTwo{4};
WPI_TalonSRX RightMotorThree{3};

// Left Side Motors
WPI_VictorSPX LeftMotorOne{10};
WPI_TalonSRX LeftMotorTwo{11};
WPI_VictorSPX LeftMotorThree{12};

// Helps Driving 
float signed_square(float x){
  return x * fabsf(x);
}
// Gyro
frc::ADXRS450_Gyro Gyro{}; 

// Cargo Intake
TalonSRX CargoIntakeMotor{2};
int Spiked = 0;

// Pneumatics
// Lift
frc::Solenoid CargoIntake{1};
bool CargoButton = false;

// HatchLock
frc::Solenoid HatchIntake{0};
bool HatchButton = false;

// Elevator Stuff
/*1 Encoder Unit = 0.0007675 inches
Encoder at 0 is ~7.75 inch off of the ground
Ball holes:
  5 | Top    | 83.5 Inches | 98697 units
  3 | Middle | 55.5 Inches | 62213 units
  1 | Bottom | 27.5 Inches | 25733 units
Hatch holes:
  4 | Top    | 75 Inches   | 87622 units
  2 | Middle | 47 Inches   | 51140 units
  0 | Bottom | 19 Inches   | 14658 units

x inches - 7.75
---------------
    0.0007675
*/

//Elevator Motors
WPI_TalonSRX ElevatorMotorOne{0};
WPI_TalonSRX ElevatorMotorTwo{13};

//Set Postitons for the Rocket (Elevator)
float EncoderHeight = 7.75;
float TopHatch = 75 - EncoderHeight;
float MiddleHatch = 47 - EncoderHeight;
float BottomHatch = 19 - EncoderHeight;
float HatchBallOffset = 7.5;
float InchesPerEncUnit = 0.0007675;
float NextPosition;
bool ElevatorButtonPressed = true;
bool ElevatorShouldAuto = false;
bool ToggleBall = false;
bool TopPOV = false;
bool RightPOV = false;
bool BottomPOV = false;
bool LeftPOV = false;

// Limit Switch 

// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

// LimeLight
//std::shared_ptr<NetworkTable> LimeTable = NetworkTable::GetTable("limelight");

// Limit Switches
frc::DigitalInput ElevatorLimitBottom{0};
frc::DigitalInput HatchLimitLeft{1};
frc::DigitalInput HatchLimitRight{2};
//                                ^all #s are subject to change...

// Variables for intake
bool threeFirstPressed = false;
float intakeCurrentStart, intakeCurrentEnd;
int intakeCurrentCounter = 0;
int intakeCurrentFrames = 5;
int intakeCurrentThreshold = 10;
bool intakeStalled = false;

// Straightens out the bot
float LastSumAngle;
float turnFact = 0.9;

/*Called on robot connection*/
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  HatchIntake.Set(false);
  CargoIntake.Set(false);

  Gyro.Reset();

  //ElevatorEnc.SetDistancePerPulse(1.037);
}

/*Called on every robot packet, no matter what mode*/
void Robot::RobotPeriodic() {

  if (ElevatorLimitBottom.Get()){
    ElevatorMotorOne.SetSelectedSensorPosition(0);
  }

  std::cout << "Encoder " << ElevatorMotorOne.GetSelectedSensorPosition() << std::endl;
  std::cout << "Next Pos " << NextPosition << std::endl;

}

/*Called when Autonomous is enabled*/
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  if (m_autoSelected == kAutoNameCustom) {
  } else {
  }
}

/*Called every robot packet in auto*/
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
  } else {
  }
}

/*Called when teleop is enabled*/
void Robot::TeleopInit() {
  
  RightMotorOne.SetInverted(true);
  RightMotorTwo.SetInverted(true);
  RightMotorThree.SetInverted(true);

  ElevatorMotorOne.SetSelectedSensorPosition(0);

  Gyro.Reset();

  NextPosition = 0;

}

/*Called every robot packet in teleop*/
void Robot::TeleopPeriodic() {
  //Gets axis for each controller
  double JoyY = JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();
  double XboxRightAnalogY = Xbox.GetRawAxis(5);

  //Power get's cut from one side of the bot to straighten out when driving straight
  float sumAngle = Gyro.GetAngle();
  float derivAngle = sumAngle - LastSumAngle;
  float correctionAngle = (sumAngle * 0.04) + (derivAngle *0.02);

  // Manual Elevator Movement
  if (XboxRightAnalogY < -0.15) {
    ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY*0.75);
		ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY*0.75);
    ElevatorShouldAuto = false;
  } else if(XboxRightAnalogY > 0.15){
    ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY*0.25);
		ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxRightAnalogY*0.25);
    ElevatorShouldAuto = false;
  } else {
    if (ElevatorShouldAuto){
      if(!(ElevatorMotorOne.GetSelectedSensorPosition() > NextPosition-5000 && ElevatorMotorOne.GetSelectedSensorPosition() < NextPosition+5000)){
        int UpOrDown = abs(ElevatorMotorOne.GetSelectedSensorPosition()-NextPosition)/(ElevatorMotorOne.GetSelectedSensorPosition()-NextPosition);
        if(UpOrDown > 0){
          ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, UpOrDown*0.1);
        ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, UpOrDown*0.1);
        } else {
          ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, UpOrDown*0.5);
          ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, UpOrDown*0.5);
        }
      } else {
        ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);
        ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);
      }
    } else {
      if(ElevatorMotorOne.GetSelectedSensorPosition() < 1000){
        ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      } else {
        ElevatorMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);
        ElevatorMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);
      }
    }
  }

  if (Xbox.GetPOV(0)){
    if(!TopPOV){
      if(ToggleBall){
        NextPosition = (TopHatch - HatchBallOffset) / InchesPerEncUnit;
      } else {
        NextPosition = (TopHatch) / InchesPerEncUnit;
      }
      TopPOV = true;
    }
  } else {
    TopPOV = false;
  }

  if (Xbox.GetPOV(90)){
    if(!RightPOV){
      ToggleBall = !ToggleBall;
      RightPOV = true;
    }
  } else {
    RightPOV = false;
  }

  if (Xbox.GetPOV(180)){
    if(!BottomPOV){
      if(ToggleBall){
        NextPosition = (BottomHatch - HatchBallOffset) / InchesPerEncUnit;
      } else {
        NextPosition = (BottomHatch) / InchesPerEncUnit;
      }
      BottomPOV = true;
    }
  } else {
    BottomPOV = false;
  }

  if (Xbox.GetPOV(270)){
    if(!LeftPOV){
      if(ToggleBall){
        NextPosition = (MiddleHatch - HatchBallOffset) / InchesPerEncUnit;
      } else {
        NextPosition = (MiddleHatch) / InchesPerEncUnit;
      }
      LeftPOV = true;
    }
  } else {
    LeftPOV = false;
  }

  if(ToggleBall){
    Xbox.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
  }

  /*// Move Elevator Up Automatically
  if (Xbox.GetRawButton(6)){
    if(!ElevatorButtonPressed) {
      NextPosition = ElevatorPositions[ElevatorPosition + 1];
      ElevatorPosition = ElevatorPosition + 1;
      ElevatorButtonPressed = true;
      ElevatorShouldAuto = true;
    }
    //Moves Elevator Down Automatically
  } else if (Xbox.GetRawButton(5)){
    if(!ElevatorButtonPressed) {
      NextPosition = ElevatorPositions[ElevatorPosition - 1];
      ElevatorPosition = ElevatorPosition - 1;
      ElevatorButtonPressed = true;
      ElevatorShouldAuto = true;
    }
  } else {
    ElevatorButtonPressed = false;
  }*/

  // Intake Lift
  if (Xbox.GetRawButton(2)){
    if (!CargoButton){
      CargoIntake.Set(!CargoIntake.Get());
      CargoButton = true;
    }
  } else {
    CargoButton = false;
  }

  // Hatch Grabber
  if (Xbox.GetRawButton(4)){
    if (!HatchButton){
      HatchIntake.Set(!HatchIntake.Get());
      HatchButton = true;
    }
  } else {
    HatchButton = false;
  }

  if (!HatchLimitLeft.Get() || !HatchLimitRight.Get()){
    HatchIntake.Set(false);
  }

  // Intakes the ball when button 3 is pressed
  if (Xbox.GetRawButton(3))
  {
    //Check if the intakeStalled variable is false, meaning that the intake motor is not currently stalling
    if (!intakeStalled)
    {

      //If the motor is not currently stalled, a counter intakeCurrentCounter is started. This counter is 3 frames long. 
      if (intakeCurrentCounter == 0) {
      
        //At the start of the counter, store the current electrical current into a variable named intakeCurrentStart
        //Also raise the counter by 1 so that this section of code only runs once and the counter is initialized.
        intakeCurrentStart = pdp.GetCurrent(2);
        intakeCurrentCounter = intakeCurrentCounter + 1;

      }
      //This next line artifically creates a delay of 3 frames, increasing the counter value by 1 with every passing frame
      else if (intakeCurrentCounter < intakeCurrentFrames) intakeCurrentCounter = intakeCurrentCounter + 1;
      //Once the counter reaches 3, three frames have passed and its now time to check the current electrical current again
      else
      {

        //After the 3 frame delay, we check for the electrical current again and store this second value in a variable named
        //intakeCurrentEnd
        //We also reset the counter back to 0 so that this continues to work as long as button 3 is being pressed
        intakeCurrentEnd = pdp.GetCurrent(2);
        intakeCurrentCounter = 0;
        
        //Now we check for the difference between the electrical current at the start of the 3 frames and at the end of the 3 frames
        //If the difference between the two is very low, in this case less than 2, then we know that the intake is either stalling
        //or that the intake is currently not intaking a ball at all.
        //The next argument in the if statement checks if the electrical current at the end of the 3 frames is greater than the
        //threshold, currently set to 10.
        //This argument lets us know that the motor is probably stalling, to differentiate it from just not intaking anything
        if ((fabs(intakeCurrentEnd - intakeCurrentStart) < 2) && intakeCurrentEnd > intakeCurrentThreshold)
        {

          /*If both of the above arguments are true, we set the intake motor to zero because it must be stalling
          We also set intakeStalled variable to true so that the whole system does not start over until button 3 is released
          and pressed again*/
          CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
          intakeStalled = true;

        }
        //If either one of the above arguements are false, it must not be stalling because of an intaked ball so it continues to spin
        //the motor
        else CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.8);

      }

    }

  }
  //When button 3 is not pressed, we set the intake motor power to zero.
  //Since the motor cannot be stalling if it's not even running, we also set the intakeStalled variable to false
  else
  { 
  
    //Spit the ball if button 1 is pressed when button 3 is not being pressed
    if (Xbox.GetRawButton(1)) {
      
      CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
      intakeStalled = false;

    }
    else 
    {

      if (intakeStalled) CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
      else CargoIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

    }

  }

  //Drive Code for CNS and modified for Axon
  //Button 5 on the wheel activates point turning
	if (RaceWheel.GetRawButton(5)) {
		RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
		RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
		RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
		LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
		Gyro.Reset();
	} 
  else {
    //Code for regular turning
	  if ((WheelX < -0.01 || WheelX > 0.01) && (JoyY > 0.06 || JoyY < -0.06)) {
		  RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
		  RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
      RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
		  LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - turnFact*(WheelX));
		  LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -JoyY - turnFact*(WheelX));
      LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -JoyY - turnFact*(WheelX));
		  Gyro.Reset();
	  }
	  //Code for driving straight
	  else if ((JoyY > 0.06 || JoyY < -0.06)) {
		  RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
		  RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
      RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
		  LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
		  LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
      LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
	  } 
    else {
	    //Dont spin any drive train motors if the driver is not doing anything
		  RightMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		  RightMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		  RightMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		  LeftMotorOne.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftMotorTwo.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftMotorThree.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  	}
	}
  
  //Straightens out bot here when driving straight
  LastSumAngle = sumAngle;

}


/*Called every robot packet in testing mode*/
void Robot::TestPeriodic() {}


/*Starts the bot*/
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
