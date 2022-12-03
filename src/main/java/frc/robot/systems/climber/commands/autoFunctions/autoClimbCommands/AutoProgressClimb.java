package frc.robot.systems.climber.commands.autoFunctions.autoClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.systems.climber.commands.GotoPositionClimberCommand;
import frc.robot.systems.climber.commands.autoFunctions.utils.AutoClimbController;
import frc.robot.systems.climber.commands.autoFunctions.utils.RollDetection;
import frc.robot.systems.climber.ClimberSubsystem;

public class AutoProgressClimb extends CommandBase{
  private ClimberSubsystem subsystem;
  private GotoPositionClimberCommand child; // The exact position command being executed
  private boolean verboseLog = true; // If to log each small thing
  private boolean stop = false; // If the command needs to stop due to a fail safe trip
  private AutoClimbController controller; // The controller for the auto climb Seq. only used if made automatically
  
  public AutoProgressClimb(ClimberSubsystem subsystem, AutoClimbController controller){
    this.subsystem = subsystem;
    this.controller = controller;
  }

  @Override
  public void initialize(){
    System.out.print("Auto picking next location ");
    if(autoCheck()) {
      abortClimb();
      return;
    }
    if(!verboseLog) System.out.println();
    // Progress the climb to the next step
    subsystem.incrementClimb();
    // Get the command for the next step
    GotoPositionClimberCommand toExecute = ((GotoPositionClimberCommand) subsystem.getNextClimbStep());
    

    // Have the command map the call path to it
    toExecute.name += " climb seq call auto timing";

    if(verboseLog) System.out.println("made automatically");

    child = toExecute;
    CommandScheduler.getInstance().schedule(toExecute);
  }

  /**
   * Checks if manual control has been started.
   * @return if currently under manual control
   */
  private boolean autoCheck(){
    if(!subsystem.underAutoControl) { // If the subsystem disagrees that it is under automatic
      if(verboseLog) System.out.println(" terminating due to manual in use");
      else System.out.println();
      stop = true;
      return true;
    }
    return false;
  }

  @Override
  public void execute(){
    if(RollDetection.isRolledOver()) {
      abortClimb();
    }
  }

  /**
   * Aborts the auto climb.
   */
  private void abortClimb(){
    if(child != null) child.cancel();
    subsystem.setPos(subsystem.getMotorPos());
    stop = true;
    controller.endAutoClimb();
  }

  @Override
  public boolean isFinished(){
    return stop || child.isFinished(); // Keep running command until stopped or child is finished
  }
}
