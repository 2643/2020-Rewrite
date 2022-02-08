package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class ValveDoubleAction
{
    public double solenoidSlideTime;
  
    private final Solenoid valveOpenSide, valveCloseSide;
    
    public ValveDoubleAction (int port)
    {

      valveOpenSide = new Solenoid (null, port);

      valveCloseSide = new Solenoid (null, port + 1);

      solenoidSlideTime = .05;

    /* This method is wired to open and pressurize the first end of the valve */

    }

    public ValveDoubleAction (PneumaticsModuleType pcmCanId, int port)
    {
      valveOpenSide = new Solenoid (pcmCanId, port);

      valveCloseSide = new Solenoid (pcmCanId, port + 1);

      solenoidSlideTime = .05;

    /* This method is wired to close and pressurize the second end of the valve */ 

    }

    public void dispose()
    {
      valveOpenSide.close();

      valveCloseSide.close();

    /* This method opens the valve and pressurizes port */

    }

    public void setFirst()
    {    
      valveCloseSide.set (false);

      valveOpenSide.set (true);

      Timer.delay (solenoidSlideTime);

      valveOpenSide.set (false);

    /* This method pressurizes the first end of the valve */

    }

    public void Extend() {
      setFirst();

    /* This method pressurizes port + 1 */

    }

  public void setSecond ()
  {
    valveOpenSide.set (false);

    valveCloseSide.set (true);

    Timer.delay(solenoidSlideTime);

    valveCloseSide.set (false);

    /* This method pressurizes the second end of the valve */

  }


  private void Retract() {

    setSecond ();

  }
  
  /* This method closes the valve */

  }
