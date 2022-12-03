package frc.robot.systems.climber;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.commands.SimulatedCommand;
import frc.robot.systems.climber.commands.autoFunctions.AutoPickPrevStep;
import frc.robot.systems.climber.commands.autoFunctions.SetupForClimb;
import frc.robot.systems.climber.commands.autoFunctions.autoClimbCommands.AutoClimb;
import frc.robot.systems.climber.commands.autoFunctions.utils.AutoClimbController;
import frc.robot.systems.climber.commands.autoFunctions.utils.ClimbSequence;
import frc.robot.systems.climber.commands.autoFunctions.utils.GetClimb;
import frc.robot.systems.climber.commands.autoFunctions.utils.RollDetection;
import frc.robot.common.dashboard.ShuffleboardHelper;
import frc.robot.common.motorfactory;
import frc.robot.systems.climber.ClimberSubsystem.ClimberPosition;
import frc.robot.systems.climber.ClimberSubsystem.ClimberSpeed;
import frc.robot.common.utilities.PitchAccessor;
import frc.robot.common.utilities.RollAccessor;
import frc.robot.systems.climber.commands.GotoPositionClimberCommand;
import frc.robot.systems.climber.commands.IncreaseManualSpeed;
import frc.robot.systems.climber.commands.ManuallyProgressClimb;
import frc.robot.systems.climber.commands.AngleTest;
import frc.robot.systems.climber.commands.DecreaseManualSpeed;
import frc.robot.systems.climber.commands.PartialDown;
import frc.robot.systems.climber.commands.PartialUp;
import frc.robot.systems.climber.commands.ZeroClimber;


public class ClimberHardware implements AutoClimbController {

  public WPI_TalonFX climberLead = null;
  public WPI_TalonFX climberFollow = null;
  public ClimberSubsystem climberSubsystem = null;

  public Command increaseManualSpeed  = new SimulatedCommand();
  public Command decreaseManualSpeed  = new SimulatedCommand();
  public Command extendClimberCommand = new SimulatedCommand();
  public Command lowerPosition        = new SimulatedCommand();
  public Command zeroClimber          = new SimulatedCommand();
  public Command upperPosition        = new SimulatedCommand();
  public Command partialDown          = new SimulatedCommand();
  public Command partialUp            = new SimulatedCommand();
  public Command traversalPause       = new SimulatedCommand();
  public Command traversalClimb       = new SimulatedCommand();
  public Command swinging             = new SimulatedCommand();
  public Command autoNext             = new SimulatedCommand();
  public Command autoPrev             = new SimulatedCommand();
  public Command autoClimb            = new SimulatedCommand();
  public Command angleTest            = new SimulatedCommand();
  public Command retract              = new SimulatedCommand();
  public Command setUpForClimb        = new SimulatedCommand();
  
  private ClimberConfig cfg = null;
  private ClimberSpeed speed;

  public ClimberHardware(ClimberConfig cfg, PitchAccessor pitchAccessor, RollAccessor rollAccessor) {
    if (cfg.simulated) return;
    setMotors(cfg);
    setDashboard(cfg);
    setCommands(cfg, pitchAccessor, rollAccessor);
    setRollDetection(cfg, rollAccessor);
    this.cfg = cfg;
  }

  private void setRollDetection(ClimberConfig cfg, RollAccessor roll){
    RollDetection.setRollAccessor(roll);
    RollDetection.setMaxRoll(cfg.maxRoll);
    RollDetection.setSubsystem(climberSubsystem);
  }

  private void setDashboard(ClimberConfig cfg){
    ShuffleboardHelper.putValueComp(cfg.upperKey, cfg.upperPosition).withWidget(BuiltInWidgets.kNumberSlider);
    ShuffleboardHelper.putValueComp(cfg.lowerKey, cfg.lowerPosition).withWidget(BuiltInWidgets.kNumberSlider);
    ShuffleboardHelper.putValueComp("trav pause", cfg.traversalPause).withWidget(BuiltInWidgets.kNumberSlider);
    ShuffleboardHelper.putValueComp("trav climb", cfg.traversalClimb).withWidget(BuiltInWidgets.kNumberSlider);
    ShuffleboardHelper.putValueComp("swinging", cfg.swinging).withWidget(BuiltInWidgets.kNumberSlider);
  }

  private void setCommands(ClimberConfig cfg, PitchAccessor pitchAccessor, RollAccessor rollAccessor){
    climberSubsystem     = new ClimberSubsystem(this, cfg);
    increaseManualSpeed  = new IncreaseManualSpeed(this.climberSubsystem, cfg);
    decreaseManualSpeed  = new DecreaseManualSpeed(this.climberSubsystem, cfg);
    extendClimberCommand = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.EXTEND, cfg);    
    lowerPosition        = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.LOWER, cfg);
    upperPosition        = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.UPPER, cfg);
    traversalPause       = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.TRAVERSAL_PAUSE, cfg);
    traversalClimb       = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.TRAVERSAL_CLIMB, cfg);
    swinging             = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.SWINGING, cfg);
    retract              = new GotoPositionClimberCommand(climberSubsystem, ClimberPosition.RETRACT, cfg);
    zeroClimber          = new ZeroClimber(this.climberSubsystem, cfg, this);
    partialDown          = new PartialDown(this.climberSubsystem, cfg);
    partialUp            = new PartialUp(this.climberSubsystem, cfg);
    autoNext             = new ManuallyProgressClimb(this.climberSubsystem);
    autoPrev             = new AutoPickPrevStep(this.climberSubsystem);
    angleTest            = new AngleTest(pitchAccessor, this.climberSubsystem, cfg);
    setUpForClimb        = new SetupForClimb(climberSubsystem, this);

    ClimbSequence seq = GetClimb.getClimb(cfg.climbSeqChoosen, cfg, this, pitchAccessor);

    Command[] climbSeq = seq.climbSeq;

    Command[] climbTimers = seq.timers;

    autoClimb = new AutoClimb(climbSeq.length-1, climbTimers, climberSubsystem, (AutoClimbController) this);

    climberSubsystem.setClimbSeq(climbSeq);
    climberSubsystem.setClimbStart(0);
    climberSubsystem.resetClimb();
  }

  private void setMotors(ClimberConfig cfg) {
    var climberLeadConfig   = new motorfactory.MotorConfiguration();
    var climberFollowConfig = new motorfactory.MotorConfiguration();

    climberLeadConfig.talonFXConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    climberLeadConfig.talonFXConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    climberLeadConfig.talonFXConfig.clearPositionOnLimitR = true;
    climberLeadConfig.talonFXConfig.slot0.kP = cfg.p;
    climberLeadConfig.talonFXConfig.slot0.kI = cfg.i;
    climberLeadConfig.talonFXConfig.slot0.kD = cfg.d;
    climberLeadConfig.talonFXConfig.slot0.kF = cfg.f;
    climberLeadConfig.invertType = cfg.leadInvert;
    
    climberFollowConfig.invertType = cfg.followInvert;

    climberLeadConfig.talonFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 60, 60, 0);
    climberLeadConfig.talonFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 00, 00, 0);

    climberFollowConfig.talonFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 60, 60, 0);
    climberFollowConfig.talonFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 0, 0, 0);

    climberLead = motorfactory.createTalonFX(cfg.leadClimberMotor, climberLeadConfig);
    climberLead.configMotionCruiseVelocity(cfg.cruiseVel);
    climberLead.configMotionAcceleration(cfg.acc);
    climberFollow = motorfactory.createTalonFX(cfg.followClimberMotor, climberFollowConfig, climberLead);
    speed = ClimberSpeed.FAST;
  }

  public void setSpeed(ClimberSpeed setSpeed){
    if(setSpeed == speed) return;
    speed = setSpeed;
    
    if(ClimberSpeed.SLOW == setSpeed){
      climberLead.configMotionCruiseVelocity(cfg.slowCruiseVel);
    }else if(ClimberSpeed.FAST == setSpeed){
      climberLead.configMotionCruiseVelocity(cfg.cruiseVel);
    }
  }

  /**
   * Used to terminate the auto climb command group.
   */
  public void endAutoClimb(){
    autoClimb.cancel();
    climberSubsystem.underAutoControl = false;
  }
}
