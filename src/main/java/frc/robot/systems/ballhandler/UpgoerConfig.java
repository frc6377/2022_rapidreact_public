package frc.robot.systems.ballhandler;

import com.fasterxml.jackson.annotation.JsonProperty;

public class UpgoerConfig {

  @JsonProperty("upgoer_left_motor_ID") // This ID is now a PWM ID, NOT a CAN ID. Ensure this is fulfilled.
  public int upgoerLeftMotorID = 12;

  @JsonProperty("upgoer_right_motor_ID")
  public int upgoerRightMotorID = 5;

  @JsonProperty("percent_output_up_upgoer")
  public double upOutput = -0.5;

  @JsonProperty("percent_output_down_upgoer")
  public double downOutput = 0.5;

  @JsonProperty("subsystem_simulated")
  public boolean simulated = false;
}