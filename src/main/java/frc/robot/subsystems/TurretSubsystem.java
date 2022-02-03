// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
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
  double leftTurnSpeed = -0.1;
  double rightTurnSpeed = 0.1;
  
  //Needs accurate values
  int LeftSoftLimit = -100000;
  int RightSoftLimit = 100000;
  //Needs testing
  double TurretP = 0;
  double TurretI = 0;
  double TurretD = 0;
  
  double TurretFF = 0;

  double TurretOutputMax = 0.5;
  double TurretOutputMin = -0.5;
  int ConversionFactor = 4096;
  int zeroPosition = 0;

  //Needs testing
  double SmartMotionP = 0;
  double SmartMotionI = 0;
  double SmartMotionD = 0;
  // double ClosedLoopError = 1;

  //Needs experimenting
  double SmartMotionFF = 0;

  //Max v(t) and a(t) during Smart Motion
  double SmartMotionMaxAcceleration = 50;
  double SmartMotionMaxVelocity = 100;

  //4096 represents 1 revolution
  int TurretPosition = 4096 * 1;

  //Motor is inverted so that when -speed will turn left and +speed will turn right
  public  static final boolean InvertMotor = true;

  /** Creates a new Turret. */
  public TurretSubsystem() 
  {
    //Motor is inverted so that when -speed will turn left and +speed will turn right
    //turretCanSparkMax.getEncoder().setInverted(InvertMotor);
    //turretCanSparkMax.getPIDController().setFF(TurretFF);
    turretCanSparkMax.getPIDController().setP(TurretP);
    turretCanSparkMax.getPIDController().setI(TurretI);
    turretCanSparkMax.getPIDController().setD(TurretD);

    turretCanSparkMax.getPIDController().setOutputRange(TurretOutputMin, TurretOutputMax);
    turretCanSparkMax.getEncoder().setPositionConversionFactor(ConversionFactor);

    // turretCanSparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(ClosedLoopError, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretPIDSlot);
    

    //turretCanSparkMax.getPIDController().setReference(0, ControlType.kSmartMotion);
  }

  public void turretCanTurn(double position)
  {
    if((LeftSoftLimit == turretCanSparkMax.getEncoder().getPosition()) || (RightSoftLimit == turretCanSparkMax.getEncoder().getPosition()))
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(position, ControlType.kSmartMotion, TurretPIDSlot);
    }
  }

  public void turretTurnLeft()
  {
    if(LeftSoftLimit <= turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(-leftTurnSpeed, ControlType.kDutyCycle, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretPIDSlot);
    }
  }

  public void turretTurnRight()
  {
    if(RightSoftLimit >= turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(-rightTurnSpeed, ControlType.kDutyCycle, TurretPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretPIDSlot);
    }
  }

  public void stopTurret()
  {
    turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle);
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

  @Override
  public void periodic() 
  {
    System.out.println(turretLimitSwitch.get());
  }
}
