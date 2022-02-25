// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class turretShoot extends CommandBase {
  /** Creates a new turretShoot. */
  boolean turretReady;
  double error;
  double pos;

  public turretShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = RobotContainer.m_turret.getPosition();
    turretReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = (double)Constants.visionTable.getEntry("Degree").getNumber(Constants.defaultVisionTurretError);
    if(error <= 1 && error >= -1){
      //turretReady = true;
      //RobotContainer.m_turret.stopTurret();
    }
    else if(error > 1){
      //RobotContainer.m_turret.turretTurnLeft();
      RobotContainer.m_turret.turretCanTurn(pos - 20);
    }
    else if(error < -1){
      //RobotContainer.m_turret.turretTurnRight();
      RobotContainer.m_turret.turretCanTurn(pos + 20);
    }
    System.out.println("Error: " + error + " Pos:" + RobotContainer.m_turret.getPosition());

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(turretReady){
    //   return true;
    // }
    // else{
    //   return false;
    // }
    return false;
  }
}
