package frc.robot.systems.signaling;

import com.fasterxml.jackson.annotation.JsonProperty;

public class SignalingConfig {
  public boolean simulated = false;

  @JsonProperty("blinkin_id")
  public static int blinkInID = 0;
}
