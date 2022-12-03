package frc.robot.systems.climber.commands.autoFunctions.autoClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.climber.commands.autoFunctions.utils.AutoClimbController;
import frc.robot.systems.climber.ClimberSubsystem;

public class AutoClimb extends SequentialCommandGroup {
  /**
   * Create a AutoClimb sequence using the timers to time the transistion betwen steps.
   * @param climbLen - the number of climber steps to go through
   * @param timers - the timers to use
   * @param subsystem - the ClimberSubsystem to use
   */
  public AutoClimb(int climbLen, Command[] timers, ClimberSubsystem subsystem, AutoClimbController controller){
    System.out.println("ClimbLen "+climbLen+" timers "+timers.length);
    addCommands(new SetAutoState(subsystem, true));
    for(int i = 0; i < timers.length; i++){
      addCommands(timers[i], new AutoProgressClimb(subsystem, controller));
    }
    System.out.println();
    for(int i = timers.length; i < climbLen; i++) {
      addCommands(new AutoProgressClimb(subsystem, controller)); 
    }
  }

  @Override
  public void end(boolean exist){
    System.out.println("Climb Complete "+exist);
  }
}
