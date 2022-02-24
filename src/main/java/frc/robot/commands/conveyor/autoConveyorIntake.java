// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import java.util.concurrent.DelayQueue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorBelt;

public class autoConveyorIntake extends CommandBase {
  /** Creates a new autoConveyorIntake. */
  
  private String initState;
  private String finalState;

  public autoConveyorIntake() {
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
        finalState = "firstonly";
      case "firstonly":
        finalState = "full";
      case "secondonly":
        finalState = "full";
      case "full":
        finalState = "full";
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(initState)
    {
      case "empty":
        RobotContainer.conveyorBelt.setSpeed(Constants.convAutoSpeed);
        if(finalState.equals(RobotContainer.conveyorBelt.getState()))
        {
          RobotContainer.conveyorBelt.turnOffConv();
          return;
        }
        break;

      case "firstonly":
        RobotContainer.conveyorBelt.setSpeed(Constants.convAutoSpeed);
        if(finalState.equals(RobotContainer.conveyorBelt.getState()))
        {
          RobotContainer.conveyorBelt.turnOffConv();
          return;
        }
        break;
      case "secondonly":
        RobotContainer.conveyorBelt.setSpeed(-Constants.convAutoSpeed);
        if("firstonly".equals(RobotContainer.conveyorBelt.getState()))
        {
          initState = "firstonly";
        }
        break;
      case "full":
        RobotContainer.conveyorBelt.turnOffConv();
        return;
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
