// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorBelt;

public class autoConveyorShoot extends CommandBase {
  /** Creates a new autoConveyorShoot. */
  
  private String initState;
  private String finalState;

  public autoConveyorShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.conveyorBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initState= RobotContainer.conveyorBelt.getState();
    switch(initState)
    {
      case "empty":
        finalState = "empty";
      case "firstonly":
        finalState = "empty";
      case "secondonly":
        finalState = "empty";
      case "full":
        finalState = "secondonly";
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(initState)
    {
      case "empty":
        RobotContainer.conveyorBelt.turnOffConv();
        return;

      case "firstonly":
        RobotContainer.conveyorBelt.setSpeed(Constants.convShooterSpeed);
        if(finalState.equals(RobotContainer.conveyorBelt.getState()))
        {
          RobotContainer.conveyorBelt.turnOffConv();
          return;
        }
        break;
      case "secondonly":
        RobotContainer.conveyorBelt.setSpeed(Constants.convShooterSpeed);
        if(finalState.equals(RobotContainer.conveyorBelt.getState()))
        {
          RobotContainer.conveyorBelt.turnOffConv();
          return;
        }
        break;
      case "full":
        RobotContainer.conveyorBelt.setSpeed(Constants.convShooterSpeed);
        if(finalState.equals(RobotContainer.conveyorBelt.getState())){
          RobotContainer.conveyorBelt.turnOffConv();
          return;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.conveyorBelt.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
