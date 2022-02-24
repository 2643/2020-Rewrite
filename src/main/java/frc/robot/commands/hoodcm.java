// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class hoodcm extends CommandBase {
  /** Creates a new Hoodlmao. */
  public hoodcm() {
    addRequirements(RobotContainer.cm_Hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.joystick.getPOV() == 0){
      RobotContainer.cm_Hood.moveUp();
    }
    else if(RobotContainer.joystick.getPOV() == 180){
      RobotContainer.cm_Hood.moveDown();
    }
    else{
      RobotContainer.cm_Hood.stopMove();
      //RobotContainer.cm_Hood.hoodPID(RobotContainer.cm_Hood.getPosition());
    }
    RobotContainer.cm_Hood.getPosition();

    //if joystick is l
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cm_Hood.stopMove();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
