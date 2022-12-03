package frc.robot.systems.climber.commands.autoFunctions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.commands.autoFunctions.utils.AutoClimbController;

public class StopAutoClimb extends CommandBase {
  private AutoClimbController c;

  public StopAutoClimb(AutoClimbController c){
    this.c = c;
  }

  @Override
  public void initialize(){
    c.endAutoClimb();
  }

  @Override
  public boolean isFinished(){
    return true;
  }

}
