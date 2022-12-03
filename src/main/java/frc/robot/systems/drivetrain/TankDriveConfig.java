package frc.robot.systems.drivetrain;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

public class TankDriveConfig {
  public boolean simulated = false;

  @JsonProperty("pidgeon_id")
  public int pigeonID = 1;

  @JsonProperty("isPigeon2")
  public boolean isPigeon2 = true;

  @JsonProperty("left_leader_id")
  public int leftLeaderID = 5;

  @JsonProperty("right_leader_id")
  public int rightLeaderID = 4;

  @JsonProperty("left_follow_id")
  public int leftFollowerID = 3;

  @JsonProperty("right_follow_id")
  public int rightFollowerID = 2;

  @JsonProperty("physical_layout")
  public PhysicalLayout physicalLayout = new PhysicalLayout();

  public class PhysicalLayout {
    @JsonProperty("pidgeon_id")
    public int pigeonID = 1;

    @JsonProperty("gear_ratio")
    public double gearRatio = 50 / 11;

    @JsonProperty("wheel_diameter_inch")
    public double wheelDiameter = 4;

    @JsonProperty("wheel_base_inch")
    public double wheelBase = 22.5;

    @JsonProperty("predicted_velocity_feet_per_second")
    public double predictedVelocity = 19.8;

    @JsonProperty("right_side_motor_is_counter_clockwise_forward")
    public boolean rightInvertIsCounterClockwise = true;
  }

  @JsonProperty("control")
  public ControlParameters controlParameters = new ControlParameters();

  public static class ControlParameters {
    @JsonProperty("velocity_feet_per_second")
    public double velocity = 16;

    @JsonProperty("high_velocity_feet_per_second")
    public double highVelocity = 30;

    @JsonProperty("error_saturation_point_feet_per_second")
    public double pSaturation = 7;

    @JsonProperty("time_to_max_velocity_seconds")
    public double accelerationTime = 1;

    @JsonProperty("turning_saturation_point_degrees")
    public double turnPSaturation = 360;

    @JsonProperty("ramsete_beta")
    // This property is derived from the Ramsete example which uses a Beta of 2.0 meters / second
    public double ramseteBeta = 78.75;

    @JsonPropertyOrder("ramsete_zeta")
    // This property is derived from the Ramsete example which uses a Zeta of 0.7 meters / second
    public double ramseteZeta = 27.5;
  }
}
