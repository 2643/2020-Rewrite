// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class JoyDrive extends CommandBase {

  /** Creates a new Joystick. */
  public JoyDrive() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.runDrivetrainRun);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    RobotContainer.runDrivetrainRun.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //System.out.println(RobotContainer.m_joystick.getRawAxis(1));
    if(Math.abs(RobotContainer.m_joystick.getRawAxis(1)) >= 0.05)
    {
      RobotContainer.runDrivetrainRun.turnLeft(RobotContainer.m_joystick.getRawAxis(1));
    }
    else
    {
      RobotContainer.runDrivetrainRun.turnLeft(0);
    }

    if(Math.abs(RobotContainer.m_joystick.getRawAxis(5)) >= 0.05)
    {
      RobotContainer.runDrivetrainRun.turnRight(RobotContainer.m_joystick.getRawAxis(5));
    }
    else
    {
      RobotContainer.runDrivetrainRun.turnRight(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
