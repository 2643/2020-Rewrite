// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyertTest extends SubsystemBase {
  TalonSRX conveyorBeltMotor = new TalonSRX(Constants.conveyorBeltMotorPort);
  /** Creates a new ConveyertTest. */
  public ConveyertTest() {
  }

  @Override
  public void periodic() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, -0.6);
  }
}
