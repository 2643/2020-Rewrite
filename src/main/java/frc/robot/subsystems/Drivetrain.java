// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase 
{ 
  CANSparkMax topleftMotor = new CANSparkMax(Constants.robotPortTopLeft, MotorType.kBrushless);
  CANSparkMax bottomleftMotor = new CANSparkMax(Constants.robotPortBottomLeft, MotorType.kBrushless);
  CANSparkMax topRightMotor = new CANSparkMax(Constants.robotPortTopRight, MotorType.kBrushless);
  CANSparkMax bottomRightMotor = new CANSparkMax(Constants.robotPortBottomRight, MotorType.kBrushless);
  // change PID values later, can't test rn
  private final double CanSparkMotorP = 0.0000005;
  private final double CanSparkMotorI= 0.00000000001;
  private final double CanSparkMotorD= 0.0000003;
  public Drivetrain() {

  topleftMotor.getPIDController().setP(CanSparkMotorP);
  topleftMotor.getPIDController().setI(CanSparkMotorI);
  topleftMotor.getPIDController().setD(CanSparkMotorD);
  topleftMotor.getPIDController().setOutputRange(-0.5, 0.5);
  topleftMotor.getEncoder().setPosition(0);
  topleftMotor.getEncoder().setPositionConversionFactor(4096);
  
  bottomleftMotor.getPIDController().setP(CanSparkMotorP);
  bottomleftMotor.getPIDController().setI(CanSparkMotorI);
  bottomleftMotor.getPIDController().setD(CanSparkMotorD);
  bottomleftMotor.getPIDController().setOutputRange(-0.5, 0.5);
  bottomleftMotor.getEncoder().setPosition(0);
  bottomleftMotor.getEncoder().setPositionConversionFactor(4096); 

  topRightMotor.getPIDController().setP(CanSparkMotorP);
  topRightMotor.getPIDController().setI(CanSparkMotorI);
  topRightMotor.getPIDController().setD(CanSparkMotorD);
  topRightMotor.getPIDController().setOutputRange(-0.5, 0.5);
  topRightMotor.getEncoder().setPosition(0);
  topRightMotor.getEncoder().setPositionConversionFactor(4096); 

  bottomRightMotor.getPIDController().setP(CanSparkMotorP);
  bottomRightMotor.getPIDController().setI(CanSparkMotorI);
  bottomRightMotor.getPIDController().setD(CanSparkMotorD);
  bottomRightMotor.getPIDController().setOutputRange(-0.5, 0.5);
  bottomRightMotor.getEncoder().setPosition(0);
  bottomRightMotor.getEncoder().setPositionConversionFactor(4096); 
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

