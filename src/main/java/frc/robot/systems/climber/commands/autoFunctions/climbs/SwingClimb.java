package frc.robot.systems.climber.commands.autoFunctions.climbs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.climber.commands.autoFunctions.timers.AngleTimer;
import frc.robot.systems.climber.commands.autoFunctions.timers.NoWaitTimer;
import frc.robot.systems.climber.commands.autoFunctions.utils.ClimbSequence;
import frc.robot.systems.climber.commands.autoFunctions.utils.DirectionOfSwing;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.systems.climber.ClimberHardware;
import frc.robot.common.utilities.PitchAccessor;
/**
 * Climbing with no partial extension
 * ACTIVATION: Manual once, Line up, AUTO
 */
public class SwingClimb extends ClimbSequence{

  public SwingClimb(ClimberConfig cfg, ClimberHardware CH, PitchAccessor pitchAccessor) {
    super(CH);

    timers = new Command[]{ 
      // Retracted -- Extend - Human
      new NoWaitTimer(),// Extend -- Hook - Human
      new NoWaitTimer(),// Pause -- Extend
      new AngleTimer(pitchAccessor, cfg.extendToSwingAngle, DirectionOfSwing.UP, cfg), //Extend -- swing
      new AngleTimer(pitchAccessor, cfg.swingToHookAngle, DirectionOfSwing.UP, cfg), // swing -- hook
      new NoWaitTimer(), // Pause -- Extend
      new AngleTimer(pitchAccessor, cfg.extendToSwingAngle, DirectionOfSwing.UP, cfg), //Extend -- swing
    };

    climbSeq = new Command[]{
      retract,
      extendClimberCommand,
      lowerPosition,
      traversalClimb,
      swinging,
      lowerPosition,
      traversalClimb,
      swinging
    }; 
  }

  
}
