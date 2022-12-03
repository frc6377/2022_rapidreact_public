package frc.robot.systems.climber.commands.autoFunctions.autoClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.utilities.PitchAccessor;

/**
 * Used for gathering data to tune climb seq
 */
public class SnapAngle extends CommandBase {
  PitchAccessor a;

  public SnapAngle(PitchAccessor a){
    this.a = a;
  }

  @Override
  public boolean isFinished(){
    return true;
  }

  @Override
  public void end(boolean nop){
    System.out.println("Current pitch is" + a.getPitch());
  }
}
