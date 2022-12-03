package frc.robot.systems.compressor;

import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticConfig {
  @JsonProperty("Simulated")
  public boolean simulated = false;

  @JsonProperty("PCM_Type")
  public PneumaticsModuleType pcmType =
      PneumaticsModuleType.REVPH; // TODO: integrate enum into config

  @JsonProperty("compressor_ID")
  public int compressorID = 0; // TODO: Actual Value
}
