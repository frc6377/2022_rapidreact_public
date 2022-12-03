package frc.robot.systems.climber.commands.autoFunctions.autoClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.systems.climber.commands.GotoPositionClimberCommand;

/**
 * Just a basic auto climb, with 10s waiting betwen each step
 * This should not be used
 * A faster more complete version is funcitoning.
 */
public class BasicAutoClimb extends SequentialCommandGroup{
  public BasicAutoClimb(Command[] climbSeq){
    for(Command c : climbSeq){
      addCommands(((GotoPositionClimberCommand) c).duplicate());
      addCommands(new WaitCommand(10));
    }
  }

  @Override
  public void end(boolean interupt){
    if(!interupt) return;
  }
}
