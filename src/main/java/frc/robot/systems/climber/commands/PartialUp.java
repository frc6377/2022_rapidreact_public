package frc.robot.systems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.systems.climber.ClimberSubsystem;
import frc.robot.systems.climber.ClimberSubsystem.ClimberDirection;

public class PartialUp extends CommandBase{
  private ClimberSubsystem subsystem;
  
  public PartialUp(ClimberSubsystem subsystem, ClimberConfig cfg){
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize(){
    subsystem.underAutoControl = false;
    subsystem.manualControl(ClimberDirection.UP);
  }

  @Override
  public void end(boolean exist){
    subsystem.setPos(subsystem.getMotorPos());
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
