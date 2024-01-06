package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Class brought to you by Matt M 😼 '23
public class LED extends SubsystemBase {
  public final PneumaticsModuleType TYPE = PneumaticsModuleType.REVPH;
  public enum LEDColor {
    OFF,
    WHITE,
    RED,
    GREEN,
    BLUE,
    PURPLE,
    YELLOW,
    ALLIANCE
  }

  private Solenoid r, g, b;
  private LEDColor currentColor;
  // private boolean flashing;
  // private double rate = 2;
  // private Timer flashTimer;

  public LED() {
    r = new Solenoid(TYPE, 3);
    g = new Solenoid(TYPE, 2);
    b = new Solenoid(TYPE, 1);

    // flashTimer = new Timer();

    setLED(LEDColor.ALLIANCE);
  }

  /**
   * 
   * @param color The color
   * @param flashing Whether it should flash
   * @param rate How many times, per second, the lights should turn on and off. Default 2.
   */
  // public void setLED(LEDColor color, boolean flashing, double rate) {
  //   currentColor = color;
  //   if (flashing && !this.flashing) { //Should only reset the flashing timer when flashing changes to true 
  //     flashTimer.restart();
  //   }
  //   this.flashing = flashing;
  //   this.rate = rate;
  //   switch (color) {
  //     case OFF :
  //       off();
  //       break;
  //     case WHITE :
  //       white();
  //       break;
  //     case RED :
  //       red();
  //       break;
  //     case BLUE :
  //       blue();
  //       break;
  //     case GREEN :
  //       green();
  //       break;
  //     case PURPLE :
  //       purple();
  //       break;
  //     case YELLOW :
  //       yellow();
  //       break;
  //     case ALLIANCE :
  //       allianceColor();
  //       break;
  //   }
  // }

  public void setLED(LEDColor color) {
    // setLED(color, flashing, rate);
    currentColor = color;
    switch (currentColor) {
      case OFF :
        off();
        break;
      case WHITE :
        white();
        break;
      case RED :
        red();
        break;
      case BLUE :
        blue();
        break;
      case GREEN :
        green();
        break;
      case PURPLE :
        purple();
        break;
      case YELLOW :
        yellow();
        break;
      case ALLIANCE :
        allianceColor();
        break;
    }
  }

  @Override
  public void periodic() {
    // if (flashing) {
    //   if (flashTimer.get() <= 1/(2*rate)) {
    //     setLED(LEDColor.OFF);
    //   } else if (flashTimer.get() <= 1/rate) {
    //     setLED(currentColor);
    //   } else {
    //     flashTimer.reset();
    //   }
    // } else {
    //   setLED(currentColor);
    // }
  }

  private void off() {
    r.set(false);
    g.set(false);
    b.set(false);
  }

  private void white() {
    r.set(true);
    g.set(true);
    b.set(true);
  }

  private void red() {
    r.set(true);
    g.set(false);
    b.set(false);
  }

  private void green() {
    r.set(false);
    g.set(true);
    b.set(false);
  }

  private void blue() {
    r.set(false);
    g.set(false);
    b.set(true);
  }

  private void yellow() {
    r.set(true);
    g.set(true);
    b.set(false);
  }

  private void purple() {
    r.set(true);
    g.set(false);
    b.set(true);
  }

  private void allianceColor() {
    switch (DriverStation.getAlliance()) {
      case Blue :
        blue();
        break;
      case Red :
        red();
        break;
      case Invalid :
        green();
        break;
    } 
  }
}