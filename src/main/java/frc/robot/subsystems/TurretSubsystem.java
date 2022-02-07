// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  CANSparkMax turretCanSparkMax = new CANSparkMax(Constants.TurretMotorPort, MotorType.kBrushless);
  DigitalInput turretLimitSwitch = new DigitalInput(10);

  int TurretPIDSlot = 0;

  int stopSpeed = 0;
  double leftTurnSpeed = -1000;
  double rightTurnSpeed = 1000;
  
  //Needs accurate values
  int LeftSoftLimit = -100000;
  int RightSoftLimit = 100000;
  //Needs testing
  double TurretSmartVelocityP = 0.0001;
  //Tested:
    //0.00001 Doesn't move
    //0.0001 Moves slowly
  double TurretSmartVelocityI = 0;
  double TurretSmartVelocityD = 0;
  
  double TurretFF = 0;

  double TurretOutputMax = 0.5;
  double TurretOutputMin = -0.5;
  int ConversionFactor = 4096;
  int zeroPosition = 0;

  //Needs experimenting
  // double SmartMotionFF = 0;
  // double ClosedLoopError = 1;

  //Max v(t) and a(t) during Smart Velocity
  double SmartMotionMaxAcceleration = 250;
  double SmartMotionMaxVelocity = 1000;

  //4096 represents 1 revolution
  int TurretPosition = 4096 * 1;

  //Motor is inverted so that when -speed will turn left and +speed will turn right
  boolean InvertMotor = true;

  /** Creates a new Turret. */
  public TurretSubsystem() 
  {
    //Motor is inverted so that when -speed will turn left and +speed will turn right
    //turretCanSparkMax.getPIDController().setFF(TurretFF);
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
    turretCanSparkMax.getPIDController().setP(TurretSmartVelocityP);
    turretCanSparkMax.getPIDController().setI(TurretSmartVelocityI);
    turretCanSparkMax.getPIDController().setD(TurretSmartVelocityD);

    turretCanSparkMax.getPIDController().setOutputRange(TurretOutputMin, TurretOutputMax);
    turretCanSparkMax.getEncoder().setPositionConversionFactor(ConversionFactor);
    turretCanSparkMax.setInverted(InvertMotor);
    turretCanSparkMax.setIdleMode(IdleMode.kCoast);

    // turretCanSparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(ClosedLoopError, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretPIDSlot);
    

    //turretCanSparkMax.getPIDController().setReference(0, ControlType.kSmartMotion);
  }
//Position is not used
  public void turretCanTurn(double position)
  {
    if((LeftSoftLimit == turretCanSparkMax.getEncoder().getPosition()) || (RightSoftLimit == turretCanSparkMax.getEncoder().getPosition()))
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(position, ControlType.kSmartVelocity, TurretPIDSlot);
    }
  }

  public void turretTurnLeft()
  {
    if(LeftSoftLimit <= turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(leftTurnSpeed, ControlType.kSmartVelocity, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity, TurretPIDSlot);
    }
  }

  public void turretTurnRight()
  {
    if(RightSoftLimit >= turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(rightTurnSpeed, ControlType.kSmartVelocity, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity, TurretPIDSlot);
    }
  }

  public void stopTurret()
  {
    turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity);
  }

  //returns true if limitswitch is hits the reflective tape and returns false otherwise
  public boolean turretLimitSwitchReflected()
  {
    return !turretLimitSwitch.get();
  }

  public void resetEncoder()
  {
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
  }

  public double getPosition()
  {
    return turretCanSparkMax.getEncoder().getPosition();
  }

  public double getVelocity()
  {
    return turretCanSparkMax.getEncoder().getVelocity();
  }

  @Override
  public void periodic() 
  {
    //System.out.println(getVelocity());
    //turretCanSparkMax.getPIDController().setReference(0.1, ControlType.kDutyCycle);
  }
}
