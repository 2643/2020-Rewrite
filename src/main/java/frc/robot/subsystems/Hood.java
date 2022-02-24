// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hood extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(Constants.motorPort, MotorType.kBrushless);
  private DigitalInput maxPositionTrue = new DigitalInput(Constants.maxHoodLimitPort);

  //sets PID variables
    private static final double kP = 0.000001;
    private static final double kI = 0.0;
    private static final double kD = 0;

    private static final double outputRange = 0;

  //max/min position that the hood can move to
  //private static final double maxPosition = 10000;
  private static final double minPosition = 0;


  /** Creates a new Hood. */
  public Hood() {
    //sets PID values
    motor.getPIDController().setP(kP);
    motor.getPIDController().setI(kI);
    motor.getPIDController().setD(kD);
    motor.getPIDController().setOutputRange(-1*(outputRange), outputRange);

  }
  //resets encoder to 0
  public void setEncoder(){
    motor.getEncoder().setPosition(0);
  }
  //stops the motion of the hood/robot is at a resting position
  public void stopMove(){
    motor.getPIDController().setReference(0, ControlType.kDutyCycle);
  }

  public boolean atTopPos(){
    return maxPositionTrue.get();
  }

  public boolean atBottomPos(){
    return (motor.getEncoder().getPosition() == minPosition);
  }

  public void hoodPID(double position){
    motor.getPIDController().setReference(position, ControlType.kPosition, 0);
  }

  //moves the hood up
  public void moveUp() { 
    if(!(atTopPos())){
      motor.getPIDController().setReference(Constants.hoodSpeed, ControlType.kDutyCycle);
    }
  }

  //moves the hood down
  public void moveDown(){
    if(!(atBottomPos())){
      motor.getPIDController().setReference(-1*(Constants.hoodSpeed), ControlType.kDutyCycle);
    }

  }

  //gets the position of the hood motor
  public void getPosition(){
    System.out.println(motor.getEncoder().getPosition());
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
