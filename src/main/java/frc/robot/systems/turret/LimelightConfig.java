package frc.robot.systems.turret;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightConfig {
  public boolean simualated = false;

  @JsonProperty("mounting_angle_close_range")
  public double mountingAngleCloseRange = -1;

  @JsonProperty("mounting_height_close_range")
  public double mountingHeightCloseRange = -1;

  @JsonProperty("mounting_angle_long_range")
  public double mountingAngleLongRange = -1;

  @JsonProperty("mounting_height_long_range")
  public double mountingHeightLongRange = -1;
}
