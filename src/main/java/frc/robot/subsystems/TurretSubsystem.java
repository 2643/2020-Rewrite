// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Turret.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  CANSparkMax turretCanSparkMax = new CANSparkMax(Constants.TurretMotorPort, MotorType.kBrushless);
  DigitalInput turretLimitSwitch = new DigitalInput(10);
  int TurretVelocityPIDSlot = 0;
  int TurretPositionPIDSlot = 1;

  int stopSpeed = 0;
  double leftTurnSpeed = -0.1;
  double rightTurnSpeed = 0.1;

  //Needs accurate values
  int LeftSoftLimit = -210000;
  int RightSoftLimit = 230000;

  int LeftHardLimit = -230000;
  int RightHardLimit = 250000;
  //Needs testing
  double TurretSmartVelocityP = 0.0001;
  //Tested:
    //0.00001 Doesn't move
    //0.0001 Moves slowly
  double TurretSmartVelocityI = 0;
  double TurretSmartVelocityD = 0;

  //double TurretSmartMotionP = 0.00004;
  double TurretSmartMotionI = 0;
  double TurretSmartMotionD = 0;
  
  double TurretFF = 0;

  double TurretOutputMax = 0.5;
  double TurretOutputMin = -0.5;
  int ConversionFactor = 4096;
  int zeroPosition = 0;

  //Max v(t) and a(t) during Smart Velocity
  double SmartVelocityMaxAcceleration = 250;
  double SmartVelocityMaxVelocity = 1000;

  double SmartMotionMaxAcceleration = 1000;
  double SmartMotionMaxVelocity = 1500;

  //4096 represents 1 revolution
  int TurretPosition = 4096 * 1;

  //Motor is inverted so that when -speed will turn left and +speed will turn right
  boolean InvertMotor = true;

  NetworkTableEntry ShuffleBoardData = Shuffleboard.getTab("TalonFX").add("P Value", 0.00005).getEntry();
  ShuffleboardTab TalonFXTab = Shuffleboard.getTab("TalonFX");
  NetworkTableEntry positionTurret = TalonFXTab.add("Position Value", 0).getEntry();
  NetworkTableEntry lol1 = TalonFXTab.add("I Value", 0.000000).getEntry();
  NetworkTableEntry lol2 = TalonFXTab.add("D Value", 0.000000).getEntry();


  /** Creates a new Turret. */
  public TurretSubsystem() {
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
    turretCanSparkMax.getPIDController().setP(TurretSmartVelocityP, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setI(TurretSmartVelocityI, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setD(TurretSmartVelocityD, TurretVelocityPIDSlot);

    // turretCanSparkMax.getPIDController().setP(TurretSmartMotionP, TurretPositionPIDSlot);
    // turretCanSparkMax.getPIDController().setI(TurretSmartMotionI, TurretPositionPIDSlot);
    // turretCanSparkMax.getPIDController().setD(TurretSmartMotionD, TurretPositionPIDSlot);

    turretCanSparkMax.getPIDController().setOutputRange(TurretOutputMin, TurretOutputMax);
    turretCanSparkMax.getEncoder().setPositionConversionFactor(ConversionFactor);
    turretCanSparkMax.setInverted(InvertMotor);
    turretCanSparkMax.setIdleMode(IdleMode.kCoast);

    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartMotionMaxAcceleration, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartMotionMaxVelocity, TurretPositionPIDSlot);

    turretCanSparkMax.getPIDController().setSmartMotionMaxAccel(SmartVelocityMaxAcceleration, TurretVelocityPIDSlot);
    turretCanSparkMax.getPIDController().setSmartMotionMaxVelocity(SmartVelocityMaxVelocity, TurretVelocityPIDSlot);

    //turretCanSparkMax.setIdleMode(IdleMode.kBrake);
  }
//Position is not used
  public void turretCanTurn(double positionValue)  {
    if(positionValue > 0) {
      if(RightSoftLimit <= getPosition()) {
        if(RightHardLimit <= getPosition()) {
          turretCanSparkMax.disable();
        }
      }
      else {
        turretCanSparkMax.getPIDController().setReference(positionValue, ControlType.kSmartMotion, TurretPositionPIDSlot);
      }
    }
    else if(positionValue < 0) {
      if(LeftSoftLimit >= getPosition()) {
        if(LeftHardLimit >= getPosition()) {
          turretCanSparkMax.disable();
        }
      }
      else {
        turretCanSparkMax.getPIDController().setReference(positionValue, ControlType.kSmartMotion, TurretPositionPIDSlot);
      }
    }
  }

  public void turretTurnLeft()  {
    if(LeftSoftLimit <= turretCanSparkMax.getEncoder().getPosition()){
      turretCanSparkMax.getPIDController().setReference(leftTurnSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
    else{
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
  }

  public void turretTurnRight() {
    if(RightSoftLimit >= turretCanSparkMax.getEncoder().getPosition()){
      turretCanSparkMax.getPIDController().setReference(rightTurnSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
    else{
      turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
    }
  }

  public void stopTurret() {
    turretCanSparkMax.getPIDController().setReference(stopSpeed, ControlType.kDutyCycle, TurretVelocityPIDSlot);
  }

  //returns true if limitswitch is hits the reflective tape and returns false otherwise
  public boolean turretLimitSwitchReflected() {
    return !turretLimitSwitch.get();
  }

  public void resetEncoder()  {
    turretCanSparkMax.getEncoder().setPosition(zeroPosition);
  }

  public double getPosition() {
    return turretCanSparkMax.getEncoder().getPosition();
  }

  public double getVelocity() {
    return turretCanSparkMax.getEncoder().getVelocity();
  }

  @Override
  public void periodic()  {
    //System.out.println(getPosition());
    //System.out.println(getVelocity());
    //turretCanSparkMax.getPIDController().setReference(0.1, ControlType.kDutyCycle);
    System.out.println(" Pos: " + getPosition() + " Error:" + (double)Constants.visionTable.getEntry("Degree").getNumber(Constants.defaultVisionTurretError));
    double PValue = ShuffleBoardData.getDouble(0.00005);
    double IValue = lol1.getDouble(0.00000);
    double DValue = lol2.getDouble(0);
    turretCanSparkMax.getPIDController().setP(PValue, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setI(IValue, TurretPositionPIDSlot);
    turretCanSparkMax.getPIDController().setD(DValue, TurretPositionPIDSlot);
    positionTurret.setDouble(getPosition());
    Constants.wantedPositionTurret.setDouble(driverControl.pos);
    //P is 0.00001: Doesn't move
    //P is 0.0001:Moves but oscilates
    //P is 0.00002: Moves 
    //P is 0.000025: Moves with less oscilation
    //P is 0.000022: Very little oscilation
    //P is 0.0000211: Much less oscilation but not exact.
  }
}
