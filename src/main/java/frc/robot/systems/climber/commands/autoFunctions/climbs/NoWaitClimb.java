package frc.robot.systems.climber.commands.autoFunctions.climbs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.climber.commands.autoFunctions.timers.NoWaitTimer;
import frc.robot.systems.climber.commands.autoFunctions.utils.ClimbSequence;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.systems.climber.ClimberHardware;
import frc.robot.common.utilities.PitchAccessor;
/**
 * Proof of climber functionality
 * ACTIVATION: Auto
 */
public class NoWaitClimb extends ClimbSequence{

  public NoWaitClimb(ClimberConfig cfg, ClimberHardware CH, PitchAccessor pitchAccessor) {
    super(CH);

    timers = new Command[]{ 
      new NoWaitTimer(), // Retracted -- Extend - Human
      new NoWaitTimer(), // Extend -- Hook - Human
      new NoWaitTimer(), // Hook -- pause
      new NoWaitTimer(), // Pause -- Extend
      new NoWaitTimer(), //Extend -- swing
      new NoWaitTimer(), // swing -- hook
      new NoWaitTimer(), // hook -- pause
      new NoWaitTimer(), // Pause -- Extend
      new NoWaitTimer(), //Extend -- swing
    };

    climbSeq = new Command[]{
      retract,
      extendClimberCommand,
      lowerPosition,
      traversalPause,
      traversalClimb,
      swinging,
      lowerPosition,
      traversalPause,
      traversalClimb,
      swinging
    }; 
  }

}
