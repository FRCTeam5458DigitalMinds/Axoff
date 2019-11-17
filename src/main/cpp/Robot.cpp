/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Include statements
#include <string>
#include <sstream>
#include <Robot.h>
#include <WPILib.h>
#include <stdlib.h>
#include <iostream>
#include <frc/Timer.h>
#include <TimedRobot.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <Encoder.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>

//Declarations

TalonSRX srx = {0};

// Pneumatics
//frc::Solenoid FeederToggle {0};

//PDP
frc::PowerDistributionPanel pdp{0};

//Joysticks
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

//Motors

//RightSideMotors
WPI_VictorSPX RightFront {13};
WPI_VictorSPX RightMid {14};
WPI_TalonSRX  RightBack {15};
//LeftSideMotors
WPI_VictorSPX LeftFront {2};
WPI_VictorSPX LeftMid {1};
WPI_TalonSRX  LeftBack {3};

//Intake motors
WPI_VictorSPX Intake {10};
WPI_VictorSPX Feeder {11};

//4 Bar
WPI_TalonSRX Right4Bar {12}; //Encoder
WPI_VictorSPX Left4Bar {7}; 

double FourBarDegrees = 0;

//One encoder rotation is 4096 units
//One encoder rotation is equal to 1/44 of full 4bar rotation
//One full 4bar rotation is 180,224 units
//One 4bar degree is 500

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  srx.Set(ControlMode::PercentOutput, 0);

  //Inverted motors
  LeftFront.SetInverted(true);
  LeftMid.SetInverted(true);
  LeftBack.SetInverted(true);

  //FeederToggle.Set(false);

  Right4Bar.SetSelectedSensorPosition(0.0);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  //Gets axis for each controller (Driving/Operating)
  double JoyY = -JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();
  double XboxLeftAnalogY = Xbox.GetRawAxis(1);

  FourBarDegrees = Right4Bar.GetSelectedSensorPosition()/500;

  //Drive code
  //Point turning
  if (RaceWheel.GetRawButton(5)){
    RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
  } else {
    //If both controls are being used
    if ((WheelX < -0.01 || WheelX > 0.01) && (JoyY > 0.06 || JoyY < -0.06)) {
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY - WheelX);
      RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY - WheelX);
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY - WheelX);
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY + WheelX);
      LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY + WheelX);
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY + WheelX);
    }
    //Only the JoyY is pressed
    else if ((JoyY > 0.1|| JoyY < -0.1)) {
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
      RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
      LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyY);
    }
    //Nothing is pressed
    else {
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      RightMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftMid.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
  }

  //4 Bar Code  
  if ((XboxLeftAnalogY > 0.1 /*&& FourBarDegrees > 5*/ ) || (XboxLeftAnalogY < -0.1 /*&& FourBarDegrees < 120*/)) { 
    Right4Bar.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxLeftAnalogY*0.3);
    Left4Bar.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, XboxLeftAnalogY*0.3);
  } else {
    Right4Bar.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.04);
    Left4Bar.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.04);
  }
  
  //Cargo intake
  //intake ball
  if (Xbox.GetRawButton(3)){
    Intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.75);
    //Feeder.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  //shoot ball
  } else if (Xbox.GetRawButton(1)){
    Intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.75);
  //stop if nothing is pressed
  } else {
    Intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    //Feeder.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }
  // toggles feeder position
  /*if (Xbox.GetRawButtonPressed(4)){
    FeederToggle.Set(!FeederToggle.Get());
  }*/
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
