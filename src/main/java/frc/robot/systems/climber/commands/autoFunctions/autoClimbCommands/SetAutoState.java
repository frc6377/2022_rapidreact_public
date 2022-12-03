package frc.robot.systems.climber.commands.autoFunctions.autoClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.ClimberSubsystem;

public class SetAutoState extends CommandBase {
  private ClimberSubsystem subsystem;
  private boolean state;
  public SetAutoState(ClimberSubsystem subsystem, boolean state){
    this.subsystem = subsystem;
    this.state = state;
  }
  
  @Override
  public void initialize(){
    subsystem.underAutoControl = state;
  }


  @Override
  public boolean isFinished(){
    return true;
  }
}
