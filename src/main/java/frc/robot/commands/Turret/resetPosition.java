// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Timer;
import java.util.TimerTask;

public class resetPosition extends CommandBase {

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
    //timer.schedule(timeForReset, 0L, 4000L);
    if(RobotContainer.m_turret.turretLimitSwitchReflected())
    {
      
      RobotContainer.m_turret.stopTurret();
      RobotContainer.m_turret.resetEncoder();
    }
    else
    {
      RobotContainer.m_turret.turretTurnLeft();
    }
  }v
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    RobotContainer.m_turret.stopTurret();
    RobotContainer.m_turret.turretPositionTurn(4096);
    RobotContainer.m_turret.resetEncoder();
    CommandScheduler.getInstance().cancel(RobotContainer.m_resetPosition);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(RobotContainer.m_turret.turretLimitSwitchReflected())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
