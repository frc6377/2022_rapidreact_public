package frc.robot.systems.climber.commands.autoFunctions.timers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.ClimberSubsystem;

public class CurrentTimer extends CommandBase{
  double trip;
  ClimberSubsystem subsystem;

  public CurrentTimer(double trip, ClimberSubsystem subsystem){
    this.subsystem = subsystem;
    this.trip = trip;
  }

  @Override
  public boolean isFinished(){
    return subsystem.currentAbove(trip);
  }
}
