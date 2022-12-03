package frc.robot.systems.turret;

import com.fasterxml.jackson.annotation.JsonProperty;

public class TurretConfig {
  public boolean simulated = false;

  @JsonProperty("turret_id")
  public int turretId = -1;

  @JsonProperty("max_velocity")
  public int maxVelo = -1;

  @JsonProperty("max_acceleration")
  public int maxAccel = -1;

  @JsonProperty("kP)")
  public double kP = -1;

  @JsonProperty("kF")
  public double kF = -1;

  @JsonProperty("tics_to_degrees")
  public double ticsToDegrees = -1;

  @JsonProperty("turret_invert_is_clockwise")
  public boolean invertIsClockwise = false;

  @JsonProperty("turret_range")
  public double turretRange = -1;

  @JsonProperty("degrees_to_tics")
  public double degreesToTics = -1;
}
