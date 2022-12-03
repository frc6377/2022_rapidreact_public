package frc.robot.systems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.common.dashboard.ShuffleboardHelper;
import frc.robot.systems.climber.ClimberSubsystem;
import frc.robot.systems.climber.ClimberSubsystem.ClimberPosition;
import frc.robot.systems.climber.ClimberSubsystem.ClimberSpeed;

public class GotoPositionClimberCommand extends CommandBase {
  public ClimberPosition targetPos;
  private ClimberSubsystem subsystem;
  private ClimberConfig cfg;
  private double pos = 0;
  public String name = "Climber goto position";

  public GotoPositionClimberCommand(ClimberSubsystem subsystem, ClimberPosition pos, ClimberConfig cfg){
    this.targetPos = pos;
    this.subsystem = subsystem;
    this.cfg = cfg;
    addRequirements(subsystem);
  }

  public GotoPositionClimberCommand duplicate(){
    return new GotoPositionClimberCommand(subsystem, targetPos, cfg);
  }

  @Override
  public void initialize(){
    switch(targetPos){
      case EXTEND:
      subsystem.setSpeed(ClimberSpeed.FAST);
      pos = cfg.extendedMotorPos;
        break;
      case SWINGING:
      subsystem.setSpeed(ClimberSpeed.SLOW);
      pos = ShuffleboardHelper.getNumberComp("swinging", cfg.swinging);
        break;
      case TRAVERSAL_CLIMB:
      subsystem.setSpeed(ClimberSpeed.FAST);
      pos = ShuffleboardHelper.getNumberComp("trav climb", cfg.traversalClimb);
        break;
      case LOWER:
      subsystem.setSpeed(ClimberSpeed.SLOW);
      pos = ShuffleboardHelper.getNumberComp(cfg.lowerKey, cfg.lowerPosition);
        break;
      case TRAVERSAL_PAUSE:
      subsystem.setSpeed(ClimberSpeed.SLOW);
      pos = ShuffleboardHelper.getNumberComp("trav pause", cfg.traversalPause);
        break;
      case UPPER:
      subsystem.setSpeed(ClimberSpeed.SLOW);
      pos = ShuffleboardHelper.getNumberComp(cfg.upperKey, cfg.upperPosition);
        break;
      case RETRACT:
      subsystem.setSpeed(ClimberSpeed.SLOW);
      pos = 0;
        break;
      default:
      System.out.println("Invalid Climber Position (GotoPositionClimberCommand:89)");
        return;
    }
    name = "Climber goto pos "+targetPos.toString()+" "+pos;
    subsystem.setPos(pos);
  }

  @Override
  public String getName(){
    return name;
  }

  @Override
  public boolean isFinished(){
    return subsystem.atPoint(pos);
  }
}
