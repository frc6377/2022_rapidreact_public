package frc.robot.systems.climber.commands.autoFunctions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.systems.climber.commands.GotoPositionClimberCommand;
import frc.robot.systems.climber.ClimberSubsystem;

public class AutoPickPrevStep extends CommandBase{
  private ClimberSubsystem subsystem;
  public boolean autoMade = false;
  
  public AutoPickPrevStep(ClimberSubsystem subsystem){
    this.subsystem = subsystem;
  }

  @Override
  public void initialize(){
    subsystem.decrementClimb();
    GotoPositionClimberCommand toExecute = ((GotoPositionClimberCommand) subsystem.getNextClimbStep());
    toExecute.name = toExecute.name + " climb seq call";
    
    if(autoMade) {
      toExecute.name += " auto timing";

      if(!subsystem.underAutoControl) return;
    } else {
      subsystem.underAutoControl = false;
    }

    CommandScheduler.getInstance().schedule(toExecute);
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
