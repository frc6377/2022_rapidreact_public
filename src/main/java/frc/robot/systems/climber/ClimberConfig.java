package frc.robot.systems.climber;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.fasterxml.jackson.annotation.JsonProperty;

import frc.robot.systems.climber.commands.autoFunctions.utils.Climbs;

public class ClimberConfig {    

  @JsonProperty("simulated")
  public boolean simulated = false;

  // ------------ Motor Config ------------

  @JsonProperty("left_climber_motor")
  public int leadClimberMotor = 7;

  @JsonProperty("right_climber_motor")
  public int followClimberMotor = 20;

  public TalonFXInvertType leadInvert = TalonFXInvertType.Clockwise;

  public TalonFXInvertType followInvert = TalonFXInvertType.FollowMaster;

  @JsonProperty("motor_P")
  public double p = 0.251082;
 
  @JsonProperty("motor_I")
  public double i = 0;

  @JsonProperty("motor_D")
  public double d = 0;
  
  @JsonProperty("motor_F")
  public double f = 0.028852;

  public double zeroingAmpLimit = 40;

  // ------------ Motion Profile and Set points ------------

  @JsonProperty("extended_motor_pos")
  public int extendedMotorPos = 241100;

  @JsonProperty("traversal_pause_position")
  public int traversalPause = 98749;

  @JsonProperty("swinging_position")
  public int swinging = 92547 ;

  @JsonProperty("traversal_climb_position")
  public int traversalClimb = 192969;

  @JsonProperty("lower_climber_position")
  public int lowerPosition = 34162 + 2700;

  @JsonProperty("upper_climber_position")
  public int upperPosition = 83938;

  @JsonProperty("climber_cruise_vel")
  public int cruiseVel = (int) (19559 * 0.9); //24576

  @JsonProperty("climber_slow_cruise_vel")
  public int slowCruiseVel = 24576/2;

  @JsonProperty("climber_acc")
  public int acc = 195994;
    
  @JsonProperty("climber_allowed_error_in_position_assesment")
  public int allowedError = 500;  

  // ------------ 'Manual' settings ------------

  @JsonProperty("manual_speed_ticks_per_100ms")
  public double manualVelocity = 1024 * 3;

  @JsonProperty("max_manual_speed")
  public double maxManualSpeed = 4096;

  @JsonProperty("minimum_manual_speed")
  public double minimumManualSpeed = 512;

  @JsonProperty("manual_speed_delta")
  public double manualSpeedChange = 512;

  @JsonProperty("climber_zero_percent")
  public double zeroingVelocity = 4096*2;

  // ------------ Auto climb settings ------------
  
  public Climbs climbSeqChoosen = Climbs.AUTO_SOONER;

  public double pauseToExtendAngle = -50.36328125;

  public double extendToSwingAngle = -52.580078125;

  public double swingToHookAngle = -36.5625;

  public double allowedAngleError = 1;

  public double maxRoll = 10;

  // ------------ Dashboard ------------

  public String upperKey = "Upper Position";

  public String lowerKey = "Lower Position";
}
