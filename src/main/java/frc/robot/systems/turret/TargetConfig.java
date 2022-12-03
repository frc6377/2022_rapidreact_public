package frc.robot.systems.turret;

import com.fasterxml.jackson.annotation.JsonProperty;

public class TargetConfig {
  public boolean simulated = false;

  @JsonProperty("target_height")
  public double targetHeight = -1;

  @JsonProperty("target_x")
  public double targetX = -1;

  @JsonProperty("target_y")
  public double targetY = -1;

  // write python script to pull pipeline from repo using below path
  /* @JsonProperty ("pipeline_path")
  public String pipelinePath = "placeholder";
  */
}
