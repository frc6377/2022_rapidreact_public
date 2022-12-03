package frc.robot.systems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.systems.climber.ClimberSubsystem;

public class ManuallyProgressClimb extends CommandBase{
  private ClimberSubsystem subsystem;
  private GotoPositionClimberCommand child; // The exact position command being executed
  private boolean verboseLog = true; // If to log each small thing
  
  public ManuallyProgressClimb(ClimberSubsystem subsystem){
    this.subsystem = subsystem;
  }

  @Override
  public void initialize(){
    // Progress the climb to the next step
    subsystem.incrementClimb();
    // Get the command for the next step
    GotoPositionClimberCommand toExecute = ((GotoPositionClimberCommand) subsystem.getNextClimbStep());
    if(verboseLog) System.out.println("Manually iterating climb");

    // Have the command map the call path to it
    toExecute.name += " manual climb seq";

    subsystem.underAutoControl = false;

    child = toExecute;
    CommandScheduler.getInstance().schedule(toExecute);
  }

  @Override
  public boolean isFinished(){
    return child.isFinished(); // Keep running command until stopped or child is finished
  }
}
