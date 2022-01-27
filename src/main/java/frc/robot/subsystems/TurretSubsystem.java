// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  CANSparkMax turretCanSparkMax = new CANSparkMax(Constants.TurretMotorPort, MotorType.kBrushless);

  public static final int TurretPIDSlot = 0;
  public static final int LeftSoftLimit = 0;
  public static final int RightSoftLimit = 0;

  public static final double TurretP = 0;
  public static final double TurretI = 0;
  public static final double TurretD = 0;
  
  public static final double TurretFF = 0;

  public static final double TurretOutputMax = 0.5;
  public static final double TurretOutputMin = -0.5;

  public static final double SmartMotionP = 0;
  public static final double SmartMotionI = 0;
  public static final double SmartMotionD = 0;
  public static final double ClosedLoopError = 1;

  public static final double SmartMotionFF = 0;

  public static final double SmartMotionMaxAcceleration = 50;
  public static final double SmartMotionMaxVelocity = 100;

  //4096 represents 1 revolution
  public static final int TurretPosition = 4096 * 1;

  /** Creates a new Turret. */
  public TurretSubsystem() 
  {
    turretCanSparkMax.getPIDController().setFF(TurretFF);
    turretCanSparkMax.getPIDController().setP(TurretP);
    turretCanSparkMax.getPIDController().setI(TurretI);
    turretCanSparkMax.getPIDController().setD(TurretD);

    turretCanSparkMax.getPIDController().setOutputRange(TurretOutputMin, TurretOutputMax);

    turretCanSparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(ClosedLoopError, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kSCurve, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretPIDSlot);

    turretCanSparkMax.getPIDController().setReference(0, ControlType.kSmartMotion);
  }

  public void turretCanTurn(double position)
  {
    if((LeftSoftLimit == turretCanSparkMax.getEncoder().getPosition()) && (RightSoftLimit == turretCanSparkMax.getEncoder().getPosition()))
    {
      turretCanSparkMax.getPIDController().setReference(0, ControlType.kDutyCycle, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(position, ControlType.kPosition, TurretPIDSlot);
    }
  }

  public void turretTurnLeft()
  {
    if(LeftSoftLimit != turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(-0.1, ControlType.kDutyCycle, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(0, ControlType.kDutyCycle, TurretPIDSlot);
    }
  }

  public void turretTurnRight()
  {
    if(RightSoftLimit != turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(0.1, ControlType.kDutyCycle, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(0, ControlType.kDutyCycle, TurretPIDSlot);
    }
  }

  public void stopTurret()
  {
    turretCanSparkMax.getPIDController().setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void periodic() 
  {
    System.out.println(turretCanSparkMax.getEncoder().getPosition());
  }
}
