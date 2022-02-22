// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorBelt extends SubsystemBase {
  /** Creates a new ConveyerBelt. */

  public static CANSparkMax conveyorBeltMotor = new CANSparkMax(Constants.conveyorBeltMotorPort, MotorType.kBrushless);
  public static DigitalInput conviRSens1 = new DigitalInput(Constants.conveyerIRSensorPort1);
  public static DigitalInput conviRSens2 = new DigitalInput(Constants.conveyerIRSensorPort2);
  public static DigitalInput[] conviRSens = {conviRSens1, conviRSens2};

  public static Boolean[] isThere = {false, false};

  public ConveyorBelt() {}

  public void setSpeed(double speed){
    conveyorBeltMotor.set(speed);
  }
// FALSE = TRUE BECAUSE FALSE = IR BLOCKED
  public void printBallsHeld()
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

  public void updateBallsHeld()
  {
    for(int i = conviRSens.length-1; i > -1; i--)
    {
      if(conviRSens[i].get() == false)
      {
        isThere[i] = false;
      }
    }
  }

  public boolean isLeft()
  {
    for(int i = 0; i < isThere.length; i++)
    {
      if(isThere[i] == false)
      {
        return true;
      }
    }
    return false;
  }

// move ot back
/* moved to intakeGo
  public void intakePrep()
  {
    if(isLeft() == true && (isThere[1]== false && isThere[0] == true))
    {
      while(conviRSens[1].get() == false)
      {
        setSpeed(Constants.convRevMotorSpeed);
      }
    }
  }
  */
  //goes until the front has at least SOMETHING
  /* moved to intakeGo
public void pushToFront()
{
  while(conviRSens[1].get() == true)
  {
    setSpeed(Constants.convIntakeSpeed);
  }
}
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateBallsHeld();
    
  }
}
