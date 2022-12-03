package frc.robot.systems.signaling;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.utilities.leds.LEDColor;
import frc.robot.common.utilities.leds.patterns.CountDownToEndOfMatch;
import frc.robot.common.utilities.leds.patterns.LEDPattern;

public class SignalingSubsystem extends SubsystemBase{
  private static SignalingSubsystem instance;
  public Spark blinkInSpark;
  private LEDColor currentColor = null;
  private ArrayList<LEDPattern> patterns = new ArrayList<>();


  private SignalingSubsystem() {
    blinkInSpark = new Spark(SignalingConfig.blinkInID);
    schedulePattern(new CountDownToEndOfMatch(0, -1));
  }

  public static SignalingSubsystem getInstance(){
    if(instance == null) instance = new SignalingSubsystem();
    return instance;
  }

  public void schedulePattern(LEDPattern pattern){
    System.out.println("Pattern Added");
    patterns.add(pattern);
    patterns.sort(LEDPattern.sorter);
  }

  public void update(){

    LEDPattern curPattern = patterns.get(0);
    long time = System.currentTimeMillis();

    if(curPattern.isFinsihed(time)){ 
      System.out.println("ended pattern"+curPattern.getClass().getSimpleName());
      patterns.remove(0);
      update();
      return;
    }


    if(curPattern.useTimeTilEndOfMatch){
      changeLEDColor(curPattern.getColor((long) DriverStation.getMatchTime() * 1000));
    }else{
      changeLEDColor(curPattern.getColor(time));
    }
  }
  
  private void changeLEDColor(LEDColor color) {
    if(color == currentColor) return;
    System.out.println("Current color:"+color.toString());
    blinkInSpark.set(color.getCode());
    currentColor = color;
  }

  @Override
  public void periodic(){
    update();
  }
}
