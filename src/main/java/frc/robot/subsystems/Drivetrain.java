// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
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
  double SmartMotionLeftMaxAccel = 2000;
  double SmartMotionRightMaxAccel = 2000;
  double SmartMotionLeftMaxVelocity = 5900;
  double SmartMotionRightMaxVelocity = 5800;


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
  topleftMotor.getPIDController().setSmartMotionMaxAccel(SmartMotionLeftMaxVelocity, PIDSlot);
  topleftMotor.getPIDController().setSmartMotionMaxVelocity(SmartMotionLeftMaxVelocity, PIDSlot);
  topleftMotor.setIdleMode(IdleMode.kCoast);
  topleftMotor.setInverted(true);
  
  bottomleftMotor.getEncoder().setPositionConversionFactor(ConversionFactor);
  bottomleftMotor.follow(topleftMotor);
   
  topRightMotor.getPIDController().setP(CanSparkMotorP);
  topRightMotor.getPIDController().setI(CanSparkMotorI);
  topRightMotor.getPIDController().setD(CanSparkMotorD);
  topRightMotor.getPIDController().setOutputRange(MinOutput, MaxOutput);
  topRightMotor.getEncoder().setPosition(resetPosition);
  topRightMotor.getEncoder().setPositionConversionFactor(ConversionFactor);
  topRightMotor.getPIDController().setSmartMotionMaxAccel(SmartMotionRightMaxAccel, PIDSlot);
  topRightMotor.getPIDController().setSmartMotionMaxVelocity(SmartMotionRightMaxVelocity, PIDSlot);
  topleftMotor.setIdleMode(IdleMode.kCoast); 

  bottomRightMotor.getEncoder().setPositionConversionFactor(ConversionFactor); 
  bottomRightMotor.follow(topRightMotor);
  }

  public void driveForward(double speed)
  {
    topleftMotor.getPIDController().setReference(speed, ControlType.kSmartVelocity, PIDSlot);
    topRightMotor.getPIDController().setReference(speed, ControlType.kSmartVelocity, PIDSlot);
  }

  public void turnLeft(double leftspeed)
  {
    topleftMotor.getPIDController().setReference(leftspeed * SmartMotionLeftMaxVelocity, ControlType.kSmartVelocity);
  }

  public void turnRight(double rightspeed)
  {
    topRightMotor.getPIDController().setReference(rightspeed * SmartMotionRightMaxVelocity , ControlType.kSmartVelocity, PIDSlot);
  }
  public void stopDrive()
  {
    topleftMotor.getPIDController().setReference(0, ControlType.kSmartVelocity, PIDSlot);
    topRightMotor.getPIDController().setReference(0, ControlType.kSmartVelocity, PIDSlot);
  }

  public void resetEncoders()
  {
    bottomleftMotor.getEncoder().setPosition(resetPosition);
    topleftMotor.getEncoder().setPosition(resetPosition);
    bottomRightMotor.getEncoder().setPosition(resetPosition);
    topRightMotor.getEncoder().setPosition(resetPosition);
  }

  public void resetRightEncoder()
  {
    topRightMotor.getEncoder().setPosition(resetPosition);
    bottomRightMotor.getEncoder().setPosition(resetPosition);
  }

  public void resetLeftEncoder()
  {
    topleftMotor.getEncoder().setPosition(resetPosition);
    bottomleftMotor.getEncoder().setPosition(resetPosition);
  }

  // public void setLeftSmartMotionControl()
  // {
  //   topleftMotor.getPIDController().setReference(0, ControlType.kSmartVelocity);
  // }

  // public void setRightSmartMotionControl()
  // {
  //   topRightMotor.getPIDController().setReference(0, ControlType.kSmartVelocity);
  // }
  public double velocity()
  {
    return topRightMotor.getEncoder().getVelocity();
  }
  public double leftvelocity()
  {
    return topleftMotor.getEncoder().getVelocity();
  }

  
  
  
  @Override
  public void periodic() 
  {
    //SmartDashboard.getNumber("Left Drive Position", topleftMotor.getEncoder().getPosition());
    //SmartDashboard.getNumber("Right Drive Position", topRightMotor.getEncoder().getPosition());
   // System.out.println("Top Left Encoder Position: " + topleftMotor.getEncoder().getPosition() + ", Bottom Left Encoder Position: " + bottomleftMotor.getEncoder().getPosition() + ", Top Right Encoder Position: " + topRightMotor.getEncoder().getPosition() + ", Bottom Right Encoder Position: " + bottomRightMotor.getEncoder().getPosition());
   //topRightMotor.getPIDController().setReference(0, ControlType.kSmartVelocity);
  }

 
}

