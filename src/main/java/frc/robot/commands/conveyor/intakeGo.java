// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorBelt;

public class intakeGo extends CommandBase {
  /** Creates a new intakeGo. */
  public intakeGo() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(RobotContainer.conveyorBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if no intake Prep button you wanna uncomment
    if(RobotContainer.conveyorBelt.isLeft() == true && (RobotContainer.conveyorBelt.isThere[1]== false && RobotContainer.conveyorBelt.isThere[0] == true))
    {
      while(RobotContainer.conveyorBelt.conviRSens[1].get() == false)
      {
        RobotContainer.conveyorBelt.setSpeed(Constants.convRevMotorSpeed);
      }
    }
    RobotContainer.conveyorBelt.pushToFront();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.conveyorBelt.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.conveyorBelt.isThere[1] == true)
    {
      return true;
    }
    return false;
  }
}
