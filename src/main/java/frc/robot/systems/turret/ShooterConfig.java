package frc.robot.systems.turret;

import com.fasterxml.jackson.annotation.JsonProperty;

public class ShooterConfig {
  @JsonProperty("shooter_invert_is_clockwise")
  public boolean shooterInvertIsClockwise = true;

  @JsonProperty("follow_invert_same_as_master")
  public boolean followMasterInvert = false;

  @JsonProperty("flywheel_id")
  public int flywheelID = -1;

  @JsonProperty("flywheelFollow_id")
  public int flywheelFollowID = -1;

  @JsonProperty("flywheel_kp")
  public double kp = 0;

  @JsonProperty("flywheel_kf")
  public double kf = 0;

  @JsonProperty("flywheel_kd")
  public double kD = 0;

  @JsonProperty("flywheel_ki")
  public double kI = 0;

  @JsonProperty("flywheel_izone")
  public double iAccum = 0;

  @JsonProperty("simulated")
  public boolean simulate = false;

  @JsonProperty("idle_velocity")
  public double idleVelo = 0;

  @JsonProperty("supply_current_limit")
  public int supplyCurrLimit = 0;

  @JsonProperty("stator_current_limit")
  public int statorCurrLimit = 0;

  @JsonProperty("hood_right_id")
  public int hoodRightID = 6;

  @JsonProperty("close_hood_pos")
  public double hoodPosCloseRange = 51.75;

  @JsonProperty("long_hood_pos")
  public double hoodPosLongRange = 270;

  @JsonProperty("zero_hood_pos")
  public double hoodPosZero = 0;

  @JsonProperty("range_with_no_target")
  public double rangeWithoutTarget = 60;
  
  @JsonProperty("rumble_velocity_tolerance")
  public double rumbleVelocityTolerence = 500;

  @JsonProperty("rumble_offset_tolerance")
  public double rumbleOffsetTolerance = 5;
  
  @JsonProperty("hood_left_id")
  public int hoodLeftID = 7;

  
}
