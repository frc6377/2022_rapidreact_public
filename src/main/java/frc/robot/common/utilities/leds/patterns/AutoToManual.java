package frc.robot.common.utilities.leds.patterns;

import frc.robot.common.utilities.leds.LEDColor;

public class AutoToManual extends LEDPattern{
  private int pulseWidth = 100;
  private int rest = 100;
  private int reps = 5;

  public AutoToManual(long start, int prioity) {
    super(start, prioity, false);
    period = (rest+pulseWidth);
  }

  @Override
  public LEDColor getColor(long time) {
    long intoSeq = getTimeIntoSequence(time) % (rest+pulseWidth);

    if(intoSeq < pulseWidth){
      return LEDColor.RED;
    }else{
      return LEDColor.OFF;
    }
  }

  @Override
  public boolean isFinsihed(long time) {
    return time > period * reps + start;
  }
  
}
