// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase 
{ 
  double CanSparkMotorP = 0.0000002;
  double CanSparkMotorI= 0.0000000000;
  double CanSparkMotorD= 0.000000;
  int PIDSlot = 0;
  double resetPosition = 0;
  double MaxOutput = 0.75;
  double MinOutput = -0.75;
  double ConversionFactor = 4096;
  double SmartMotionMaxAccel = 250;
  double SmartMotionMaxVelocity = 125;


  CANSparkMax topleftMotor = new CANSparkMax(Constants.robotPortTopLeft, MotorType.kBrushless);
  CANSparkMax bottomleftMotor = new CANSparkMax(Constants.robotPortBottomLeft, MotorType.kBrushless);
  CANSparkMax topRightMotor = new CANSparkMax(Constants.robotPortTopRight, MotorType.kBrushless);
  CANSparkMax bottomRightMotor = new CANSparkMax(Constants.robotPortBottomRight, MotorType.kBrushless);
  // change PID values later, can't test rn
  
  public Drivetrain() {

  topleftMotor.getPIDController().setP(CanSparkMotorP);
  topleftMotor.getPIDController().setI(CanSparkMotorI);
  topleftMotor.getPIDController().setD(CanSparkMotorD);
  topleftMotor.getPIDController().setOutputRange(MinOutput, MaxOutput);
  topleftMotor.getEncoder().setPosition(resetPosition);
  topleftMotor.getEncoder().setPositionConversionFactor(ConversionFactor);
  topleftMotor.getPIDController().setReference(0, ControlType.kSmartMotion);
  topleftMotor.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAccel, PIDSlot);
  topleftMotor.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, PIDSlot);
  
  bottomleftMotor.getEncoder().setPositionConversionFactor(ConversionFactor);
  bottomleftMotor.follow(topleftMotor);
   
  topRightMotor.getPIDController().setP(CanSparkMotorP);
  topRightMotor.getPIDController().setI(CanSparkMotorI);
  topRightMotor.getPIDController().setD(CanSparkMotorD);
  topRightMotor.getPIDController().setOutputRange(MinOutput, MaxOutput);
  topRightMotor.getEncoder().setPosition(resetPosition);
  topRightMotor.getEncoder().setPositionConversionFactor(ConversionFactor);
  topRightMotor.getPIDController().setReference(0, ControlType.kSmartMotion);
  topleftMotor.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAccel, PIDSlot);
  topleftMotor.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, PIDSlot); 

  bottomRightMotor.getEncoder().setPositionConversionFactor(ConversionFactor); 
  bottomRightMotor.follow(topRightMotor);
  }

  public void driveForward(double speed)
  {
    topleftMotor.getPIDController().setReference(speed, ControlType.kDutyCycle);
    topRightMotor.getPIDController().setReference(speed, ControlType.kDutyCycle);
  }

  public void turnLeft(double leftspeed)
  {
    topleftMotor.getPIDController().setReference(leftspeed, ControlType.kDutyCycle);
  }

  public void turnRight(double rightspeed)
  {
    topRightMotor.getPIDController().setReference(rightspeed, ControlType.kDutyCycle);
  }

  public void resetEncoders()
  {
    bottomleftMotor.getEncoder().setPosition(resetPosition);
    topleftMotor.getEncoder().setPosition(resetPosition);
    bottomRightMotor.getEncoder().setPosition(resetPosition);
    topRightMotor.getEncoder().setPosition(resetPosition);
  }
  
  @Override
  public void periodic() 
  {
    SmartDashboard.getNumber("Left Drive Position", topleftMotor.getEncoder().getPosition());
    SmartDashboard.getNumber("Right Drive Position", topRightMotor.getEncoder().getPosition());
    System.out.println("Top Left Encoder Position: " + topleftMotor.getEncoder().getPosition() + ", Bottom Left Encoder Position: " + bottomleftMotor.getEncoder().getPosition() + ", Top Right Encoder Position: " + topRightMotor.getEncoder().getPosition() + ", Bottom Right Encoder Position: " + bottomRightMotor.getEncoder().getPosition());
  }

 
}

