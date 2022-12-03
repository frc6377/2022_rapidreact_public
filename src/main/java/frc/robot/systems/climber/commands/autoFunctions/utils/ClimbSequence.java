package frc.robot.systems.climber.commands.autoFunctions.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.climber.ClimberHardware;

public abstract class ClimbSequence {
  public Command[] climbSeq = {};
  public Command[] timers = {};

  protected static Command extendClimberCommand;
  protected static Command traversalClimb;
  protected static Command traversalPause;
  protected static Command swinging;
  protected static Command lowerPosition;
  protected static Command retract;  

  
  protected ClimbSequence(ClimberHardware CH){
    extendClimberCommand = CH.extendClimberCommand;
    traversalClimb = CH.traversalClimb;
    traversalPause = CH.traversalPause;
    swinging = CH.swinging;
    lowerPosition = CH.lowerPosition;
    retract = CH.retract;
  }
}
