package frc.robot.common.utilities.leds.patterns;

import java.util.Comparator;

import frc.robot.common.utilities.leds.LEDColor;

public abstract class LEDPattern {
  protected long start;
  protected long period;
  protected int prioity;
  public boolean useTimeTilEndOfMatch = false;
  public static final LEDPatternSorter sorter = new LEDPatternSorter();

  public LEDPattern(long start, int prioity, boolean useTimeTilEndOfMatch){
    this.useTimeTilEndOfMatch = useTimeTilEndOfMatch;
    this.start = start;
    this.prioity = prioity;
  }

  protected long getTimeIntoSequence(long time){
    return (time-start) % period;
  }

  public int getPrioity(){
    return prioity;
  };

  public abstract LEDColor getColor(long time);

  public abstract boolean isFinsihed(long time);

  private static class LEDPatternSorter implements Comparator<LEDPattern>{
    @Override
    public int compare(LEDPattern arg0, LEDPattern arg1) {
      return arg1.prioity - arg0.prioity;
    }
  }
}
