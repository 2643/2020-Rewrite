// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax leftShooter = new CANSparkMax(Constants.leftShooterPort, MotorType.kBrushless);
  CANSparkMax rightShooter = new CANSparkMax(Constants.rightShooterPort, MotorType.kBrushless);

  double motorP = 0.0018;
  double motorI = 0.00000;
  double motorD = 0.00000; 
  double outputMin = -0.9;
  double outputMax = 0.9;
  int PIDSlot = 0;
  boolean Invertleft = true;
  boolean Invertrightfollow = true;

  public Shooter() {
    leftShooter.getPIDController().setP(motorP, PIDSlot);
    leftShooter.getPIDController().setI(motorI, PIDSlot);
    leftShooter.getPIDController().setD(motorD, PIDSlot);
    leftShooter.getPIDController().setOutputRange(outputMin, outputMax);
    leftShooter.setInverted(Invertleft);
    //leftShooter.getPIDController().setSmartMotionMaxVelocity(500, 0);
    //500 = 0.1 kDutyCycle
    //leftShooter.getEncoder().setVelocityConversionFactor(1);

    rightShooter.follow(leftShooter, Invertrightfollow);
  }

  public void setSpeed(double motorSpeed)
  {
    leftShooter.getPIDController().setReference(motorSpeed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //leftShooter.getPIDController().setReference(0.1, ControlType.kDutyCycle);
    System.out.println(leftShooter.getEncoder().getVelocity() + "    " + leftShooter.getEncoder().getVelocityConversionFactor());
  }
}
