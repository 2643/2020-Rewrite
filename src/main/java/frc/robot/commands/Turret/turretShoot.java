// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import java.util.concurrent.TimeUnit;

public class turretShoot extends CommandBase {
  /** Creates a new turretShoot. */
  boolean turretReady;
  double error;
  double pos;
  double gain = 2890;
  public static double target;

  

  public turretShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = RobotContainer.m_turret.getPosition();
    turretReady = false;
    Robot.canDriverControl = false;
    error = ((double)Constants.visionTable.getEntry("Degree").getNumber(Constants.defaultVisionTurretError));
    target = pos - error*gain;
    RobotContainer.m_turret.turretCanTurn(target);
    turretReady = true;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = RobotContainer.m_turret.getPosition();
    error = ((double)Constants.visionTable.getEntry("Degree").getNumber(Constants.defaultVisionTurretError));
    // if(error <= 5 && error >= -5){
    //   turretReady = true;
    // }
    // else if(error > 5){
    //   target = pos - 2500;
    //   RobotContainer.m_turret.turretCanTurn(target);
    // }
    // else if(error < -5){
    //   target = pos + 2500;
    //   RobotContainer.m_turret.turretCanTurn(target);
    // }
    

    System.out.println("Error: " + error + " Pos:" + RobotContainer.m_turret.getPosition() + " Ready: " + turretReady + " Target:" + target);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Robot.canDriverControl = true;
    // double posRN = RobotContainer.m_turret.getPosition();
    // RobotContainer.m_turret.turretCanTurn(posRN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turretReady){
      return true;
    }
    else{
      return false;
    }
  }
}
