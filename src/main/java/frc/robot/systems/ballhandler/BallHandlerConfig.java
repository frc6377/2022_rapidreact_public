package frc.robot.systems.ballhandler;

import com.fasterxml.jackson.annotation.JsonProperty;
import frc.robot.common.config.ColorSensorConfig;

public class BallHandlerConfig {
  public boolean simulated = false;

  @JsonProperty("front_flywheels_and_belt_motor_ID")
  public int frontFlywheelsBeltMotorID = 36;

  // Not to be used.
  @JsonProperty("back_flywheels_and_belt_motor_ID")
  public int backFlywheelsBeltMotorID = 15;

  // Arbitrary, likely low values; test to improve.
  @JsonProperty("velocity_forward_units_per_100ms")
  public double forwardVelocity = -1000;

  @JsonProperty("velocity_reverse_units_per_100ms")
  public double reverseVelocity = 1000;

  // Temporary values.
  @JsonProperty("temporary_forward_output")
  public double forwardOutput = 0.3;

  @JsonProperty("temporary_reverse_output")
  public double reverseOutput = -0.3;

  @JsonProperty("blinkin_id")
  public int blinkInID = 0;

  public ColorSensorConfig colorConfig = new ColorSensorConfig();
}
