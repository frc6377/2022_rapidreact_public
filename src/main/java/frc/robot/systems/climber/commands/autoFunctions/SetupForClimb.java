package frc.robot.systems.climber.commands.autoFunctions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.systems.climber.commands.ManuallyProgressClimb;
import frc.robot.systems.climber.commands.autoFunctions.utils.AutoClimbController;
import frc.robot.systems.climber.ClimberSubsystem;

public class SetupForClimb extends CommandBase {
  ClimberSubsystem subsystem;
  AutoClimbController controller;

  public SetupForClimb(ClimberSubsystem subsystem, AutoClimbController controller){
    this.subsystem = subsystem;
    this.controller = controller;
  }

  @Override
  public void initialize(){
    controller.endAutoClimb();
    subsystem.resetClimb();
    CommandScheduler.getInstance().schedule(new ManuallyProgressClimb(subsystem));
  }
  
  @Override
  public boolean isFinished(){
    return true;
  }
}
