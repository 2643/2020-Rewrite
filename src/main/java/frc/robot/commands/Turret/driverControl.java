// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class driverControl extends CommandBase {
  public static double posDriverControl;
  private static double target;
  /** Creates a new driverControl. */
  public driverControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    posDriverControl = RobotContainer.m_turret.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    posDriverControl = RobotContainer.m_turret.getPosition();

    if(RobotContainer.driveStick.getPOV() == 270) {
      target = posDriverControl - 200;
      RobotContainer.m_turret.turretCanTurn(target);
    }
    else if(RobotContainer.driveStick.getPOV() == 90) {
      target = posDriverControl + 200;
      RobotContainer.m_turret.turretCanTurn(target);
    }
    else
    {
      RobotContainer.m_turret.setEncoder(target);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
