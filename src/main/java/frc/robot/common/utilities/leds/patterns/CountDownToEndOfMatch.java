package frc.robot.common.utilities.leds.patterns;

import frc.robot.common.utilities.leds.LEDColor;

public class CountDownToEndOfMatch extends LEDPattern{
  private int pulseWidth = 250;

  public CountDownToEndOfMatch(long start, int prioity){
    super(start, prioity, true);
  }

  /**
   * The current color to display.
   */
  public LEDColor getColor(long time) {
    if(time < 0){
      return LEDColor.OFF;
    }

    if(!useTimeTilEndOfMatch){
      if(time%1000 < pulseWidth){
        return LEDColor.YELLOW;
      }
      return LEDColor.OFF;    
    }else if(time < 10000){
      // final 10s
      useTimeTilEndOfMatch = false;
      return LEDColor.OFF;
    }else if(time < 20000){
      // final 20s
      return LEDColor.BLUE;
    }else if(time < 30000){
      // final 30s
      return LEDColor.GREEN;
    }
    return LEDColor.OFF;
  }

  public boolean isFinsihed(long time){
    return false;
  }
}
