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
  public static DigitalInput[] conviRSens = { conviRSens1, conviRSens2 };

  public static int[] pos = { 0, 0 };

  public ConveyorBelt() {
  }

  public void setSpeed(double speed) {
    conveyorBeltMotor.set(speed);
  }

  // FALSE = TRUE BECAUSE FALSE = IR BLOCKED
  public void printBallsHeld() {

    for (int x = 0; x < pos.length; x++) {
      if (pos[x] == 1) {
        System.out.print("X");
      } else {
        System.out.print("O");
      }
    }
    System.out.println("");
  }

  public String getState() {
    updateBallsHeld();
    if(pos[0] ==0 && pos[1] == 0)
    {
      return "empty";
    }
    else if(pos[0] ==1 && pos[1] == 0)
    {
      return "firstonly";
    }
    else if(pos[0] ==0 && pos[1] == 1)
    {
      return "secondonly";
    }
    else if(pos[0] ==1 && pos[1] == 1)
    {
      return "full";
    }
    else
    {
      return "error";
    }
  }

  public void updateBallsHeld() {
    for (int i = conviRSens.length - 1; i > -1; i--) {
      if (conviRSens[i].get() == false) {
        pos[i] = 1;
      } else {
        pos[i] = 0;
      }
    }
  }

  /*
   * verifies state
   *            pos1  pos2
   * state 1 -  0     0
   * state 2 -  1     0
   * state 3 -  0     1
   * state 4 -  1     1
   */
  public boolean equalTo(String inp) {
    switch (inp) {
      case "empty":
        if (pos[0] == 0 && pos[1] == 0) {
          return true;
        }
        break;
      case "firstonly":
        if (pos[0] == 1 && pos[1] == 0) {
          return true;
        }
        break;
      case "secondonly":
        if (pos[0] == 0 && pos[1] == 1) {
          return true;
        }
        break;
      case "full":
        if (pos[0] == 1 && pos[1] == 1) {
          return true;
        }
        break;
    }
    return false;
  }

  public boolean isLeft() {
    for (int i = 0; i < pos.length; i++) {
      if (pos[i] == 1) {
        return true;
      }
    }
    return false;
  }

  public boolean intakeCheck() {
    // if it is full (full completley)
    if (equalTo("full")) {
      return false;
    }
    // until it goes from secondonly to firstonly in pos then no intake
    else if (equalTo("secondonly")) {
      while (conviRSens1.get() == true) {
        setSpeed(Constants.convRevMotorSpeed); // sets in position to firstonly
      }
      return true;
    } else {
      return true;
    }
  }

  public boolean shooterCheck() {
    // if it is empty
    if (equalTo("empty")) {
      return false;
    }
    // cant shoot until from firstonly to secondonly positioning
    else if (equalTo("firstonly")) {
      while (conviRSens2.get() == true) {
        setSpeed(Constants.convMotorSpeed);
      }
      return true;
    } else {
      return true;
    }
  }

  public void turnOffConv()
  {
    setSpeed(0);
  }
  // move ot back
  /*
   * moved to intakeGo
   * public void intakePrep()
   * {
   * if(isLeft() == true && (isThere[1]== false && isThere[0] == true))
   * {
   * while(conviRSens[1].get() == false)
   * {
   * setSpeed(Constants.convRevMotorSpeed);
   * }
   * }
   * }
   */
  // goes until the front has at least SOMETHING
  /*
   * moved to intakeGo
   * public void pushToFront()
   * {
   * while(conviRSens[1].get() == true)
   * {
   * setSpeed(Constants.convIntakeSpeed);
   * }
   * }
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateBallsHeld();

  }
}
