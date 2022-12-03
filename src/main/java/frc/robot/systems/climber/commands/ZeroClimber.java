package frc.robot.systems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.commands.autoFunctions.utils.AutoClimbController;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.systems.climber.ClimberSubsystem;

/**
 * Move the climber to zero using a combination of the limit switch and current.
 */
public class ZeroClimber extends CommandBase{
  private ClimberSubsystem subsystem;
  private ClimberConfig cfg;
  private long startCurCheck;
  private AutoClimbController controller;
  
  public ZeroClimber(ClimberSubsystem subsystem, ClimberConfig cfg, AutoClimbController controller){
    this.subsystem = subsystem;
    this.cfg = cfg;
    this.controller = controller;
  }

  public void initialize(){
    startCurCheck = System.currentTimeMillis() + 5000; // Don't start current check for 2 seconds
    subsystem.setVelocity(-cfg.zeroingVelocity);
  }

  @Override
  public void end(boolean interupted){
    subsystem.setPosition(0);
    controller.endAutoClimb();

    if(interupted) return; 
    subsystem.resetClimb();
    subsystem.setZero();

  }

  @Override
  public boolean isFinished(){
    if(startCurCheck < System.currentTimeMillis()) { // After the wait period start checking the current
      return subsystem.currentAbove(cfg.zeroingAmpLimit) || subsystem.isLimitSwitchHit();
    }
    return subsystem.isLimitSwitchHit();
  }
}
