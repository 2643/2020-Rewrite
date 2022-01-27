// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorBelt extends SubsystemBase {
  /** Creates a new ConveyerBelt. */

  public static boolean isBall = false;
  public static WPI_TalonSRX conveyorBeltMotor = new WPI_TalonSRX(Constants.conveyorBeltMotorPort);
  public static DigitalInput conviRSens1 = new DigitalInput(Constants.conveyerIRSensorPort1);
  public static DigitalInput conviRSens2 = new DigitalInput(Constants.conveyerIRSensorPort2);
  public static DigitalInput conviRSens3 = new DigitalInput(Constants.conveyerIRSensorPort3);
  public static DigitalInput conviRSens4 = new DigitalInput(Constants.conveyerIRSensorPort4);
  public static DigitalInput conviRSens5 = new DigitalInput(Constants.conveyerIRSensorPort5);
  public static DigitalInput conviRSens6 = new DigitalInput(Constants.conveyerIRSensorPort6);
  public static DigitalInput conviRSens7 = new DigitalInput(Constants.conveyerIRSensorPort7);
  public static DigitalInput conviRSens8 = new DigitalInput(Constants.conveyerIRSensorPort8);
  public static DigitalInput conviRSens9 = new DigitalInput(Constants.conveyerIRSensorPort9);
  public static DigitalInput[] conviRSens = {conviRSens1, conviRSens2, conviRSens3, conviRSens4, conviRSens5, conviRSens6, conviRSens7, conviRSens8, conviRSens9};

  public static Boolean[] isThere = {false, false, false, false, false, false, false, false, false};

  public ConveyorBelt() {}

  public void setSpeed(double speed){
    conveyorBeltMotor.set(speed);
  }

  public void updateBallsHeld()
  {
    
    for(int x = 0; x < isThere.length; x++)
    {
      if(isThere[x] == false)
      {
        System.out.print("X");
      }
      else{
        System.out.print("O");
      }
    }
    System.out.println("");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isBall = false;
    for(int x = conviRSens.length-2; x >0 ; x--)
    if(conviRSens[x].get() == false || conviRSens[x+1].get() == false)
    {
      isThere[x] = true;
      isThere[x+1] = true;
      isBall = true;
    }
    else
    {
      isThere[x] = false;
      isThere[x] = false;
    }

  }
}
