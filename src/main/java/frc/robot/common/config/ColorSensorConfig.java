package frc.robot.common.config;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

// This is for a color sensor. You can have multiple of these on a robot.
public class ColorSensorConfig {
  @JsonProperty("simulated")
  public boolean simulated = false;

  @JsonProperty("port")
  @JsonFormat(shape = JsonFormat.Shape.NUMBER_INT)
  public Port port = I2C.Port.kOnboard; // OnBoard is 0. MXP is 1.
}
