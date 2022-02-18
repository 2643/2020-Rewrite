// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {

  /** Creates a new Intake. */
  TalonSRX intakeMotor = new TalonSRX (0);
  
  private static DigitalInput intakeBall = new DigitalInput (Constants.intakeBallChannel);

  public Intake () {

    intakeMotor.setNeutralMode (NeutralMode.Coast);

  }

  public void setSpeed (double speed) {

    intakeMotor.set (TalonSRXControlMode.PercentOutput, speed);

  }

  public void isBallThere () {

    if (intakeBall.get () == false) {

	    Constants.intakingBall = true;

    }

    else {

	    Constants.intakingBall = false;
      
    }

  }

  @Override public void periodic () {
     
    isBallThere();
    
    // This method will be called once per scheduler run
  }

  public void spinClockwise() {

    intakeMotor.setInverted(false); 

  }

  public void spinCounterClockwise() {

    intakeMotor.setInverted(true);

  }
}
