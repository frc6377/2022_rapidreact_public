package frc.robot.systems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.systems.climber.ClimberSubsystem;

public class IncreaseManualSpeed extends CommandBase{
  private ClimberConfig cfg;
  private ClimberSubsystem subsystem;

  public IncreaseManualSpeed(ClimberSubsystem subsystem, ClimberConfig cfg){
    this.cfg = cfg;
    this.subsystem = subsystem;
  }

  @Override
  public void initialize(){
    subsystem.changeManualSpeed(cfg.manualSpeedChange);
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
