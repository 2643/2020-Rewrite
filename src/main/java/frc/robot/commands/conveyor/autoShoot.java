// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorBelt;

public class autoShoot extends CommandBase {
  /** Creates a new autoShoot. */

  private boolean finished = false;

  public autoShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.conveyorBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // only goes if there are balls actually in conveyor
    if(RobotContainer.conveyorBelt.isLeft() == true)
    {
      RobotContainer.conveyorBelt.setSpeed(Constants.convAutoSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.conveyorBelt.setSpeed(0);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    finished = !RobotContainer.conveyorBelt.isLeft();
    return finished;
  }
}
