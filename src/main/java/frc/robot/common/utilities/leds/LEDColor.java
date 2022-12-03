package frc.robot.common.utilities.leds;

public enum LEDColor {
  OFF(0.99),
  RED(0.61),
  YELLOW(0.69),
  GREEN(0.77),
  BLUE(0.87);
  
  private double code = -1;
  LEDColor(double codE){
    code = codE;
  }
  
  public double getCode(){
    return code;
  }
}
