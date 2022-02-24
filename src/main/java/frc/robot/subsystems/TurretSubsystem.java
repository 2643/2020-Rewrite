// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  CANSparkMax turretCanSparkMax = new CANSparkMax(Constants.TurretMotorPort, MotorType.kBrushless);
  DigitalInput turretLimitSwitch = new DigitalInput(10);

  int TurretVelocityPIDSlot = 0;
  int TurretPositionPIDSlot = 1;

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

  double TurretSmartMotionP = 0.0001;
  double TurretSmartMotionI = 0;
  double TurretSmartMotionD = 0;
  
  double TurretFF = 0;

  double TurretOutputMax = 0.5;
  double TurretOutputMin = -0.5;
  int ConversionFactor = 4096;
  int zeroPosition = 0;

  //Needs experimenting
  // double SmartMotionFF = 0;
  // double ClosedLoopError = 1;

  //Max v(t) and a(t) during Smart Velocity
  double SmartVelocityMaxAcceleration = 250;
  double SmartVelocityMaxVelocity = 1000;

  double SmartMotionMaxAcceleration = 250;
  double SmartMotionMaxVelocity = 1000;

  //4096 represents 1 revolution
  int TurretPosition = 4096 * 1;

  //Motor is inverted so that when -speed will turn left and +speed will turn right
  boolean InvertMotor = true;

  /** Creates a new Turret. */
  public TurretSubsystem() 
  {
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
    turretCanSparkMax.getPIDController().setP(TurretSmartVelocityP, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setI(TurretSmartVelocityI, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setD(TurretSmartVelocityD, TurretVelocityPIDSlot);

    turretCanSparkMax.getPIDController().setP(TurretSmartMotionP, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setI(TurretSmartMotionI, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setD(TurretSmartMotionD, TurretPositionPIDSlot);

    turretCanSparkMax.getPIDController().setOutputRange(TurretOutputMin, TurretOutputMax);
    turretCanSparkMax.getEncoder().setPositionConversionFactor(ConversionFactor);
    turretCanSparkMax.setInverted(InvertMotor);
    turretCanSparkMax.setIdleMode(IdleMode.kCoast);

    turretCanSparkMax.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretPositionPIDSlot);

    turretCanSparkMax.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretVelocityPIDSlot);
    
  }
//Position is not used
  public void turretCanTurn(double position)
  {
    if((LeftSoftLimit >= turretCanSparkMax.getEncoder().getPosition()) || (RightSoftLimit <= turretCanSparkMax.getEncoder().getPosition()))
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartMotion, TurretPositionPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(position, ControlType.kSmartMotion, TurretPositionPIDSlot);
    }
  }

  public void turretTurnLeft()
  {
    if(LeftSoftLimit <= turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(leftTurnSpeed, ControlType.kSmartVelocity, TurretVelocityPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity, TurretVelocityPIDSlot);
    }
  }

  public void turretTurnRight()
  {
    if(RightSoftLimit >= turretCanSparkMax.getEncoder().getPosition())
    {
      turretCanSparkMax.getPIDController().setReference(rightTurnSpeed, ControlType.kSmartVelocity, TurretVelocityPIDSlot);
    }
    else
    {
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity, TurretVelocityPIDSlot);
    }
  }

  public void stopTurret()
  {
    turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kSmartVelocity, TurretVelocityPIDSlot);
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
<<<<<<< HEAD
    System.out.println(getPosition());
=======
    //System.out.println(getVelocity());
>>>>>>> 0994c933122fb2ff0b8645ec32bf22cd9204512f
    //turretCanSparkMax.getPIDController().setReference(0.1, ControlType.kDutyCycle);
  }
}
