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

  double motorP = 0;
  double motorI = 0;
  double motorD = 0; 
  double outputMin = -0.5;
  double outputMax = 0.5;
  int PIDSlot = 0;
  boolean Invertright = true;

  public Shooter() {
    leftShooter.getPIDController().setP(motorP, PIDSlot);
    leftShooter.getPIDController().setI(motorI, PIDSlot);
    leftShooter.getPIDController().setD(motorD, PIDSlot);
    leftShooter.getPIDController().setOutputRange(outputMin, outputMax);

    rightShooter.follow(leftShooter);
    rightShooter.setInverted(Invertright);
  }

  public void setSpeed(double speed)
  {
    leftShooter.getPIDController().setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
