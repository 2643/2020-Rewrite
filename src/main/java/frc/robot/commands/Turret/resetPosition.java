// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class resetPosition extends CommandBase {
  double pos;
  double target;

  /** Creates a new resetPosition. */    
  public resetPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //timer.schedule(timeForReset, 0L, 4000L);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    pos = RobotContainer.m_turret.getPosition();

    if(!RobotContainer.m_turret.turretLimitSwitchReflected()) {
      target = pos - 3500;
      RobotContainer.m_turret.turretCanTurn(target);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_turret.setEncoder(0);
    RobotContainer.m_turret.turretCanTurn(0);
    //RobotContainer.m_turret.turretCanTurn(4096);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(RobotContainer.m_turret.turretLimitSwitchReflected()) {
      return true;
    }
    else {
      return false;
    }
  }
}
