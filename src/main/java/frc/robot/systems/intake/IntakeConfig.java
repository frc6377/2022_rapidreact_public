package frc.robot.systems.intake;

import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.systems.compressor.PneumaticConfig;

public class IntakeConfig {
  public PneumaticConfig PC;
  public PneumaticsModuleType pcmType = null; // Defined in pneumatic config

  @JsonProperty("simulated")
  public boolean simulated = false;

  @JsonProperty("roller_id")
  public int rollerId = 0;

  @JsonProperty("pneumatic_control_canID")
  public int pneumaticControllerID = 1;

  @JsonProperty("soleniod_extension")
  public int soleniodExtensionModuleId = 0;

  @JsonProperty("soleniod_rectraction")
  public int soleniodRetrationModuleId = 0;

  @JsonProperty("intake_percentage")
  public double intakePercentage = 0.5;

  @JsonProperty("eject_percentage")
  public double ejectPercentage = -0.5;

  @JsonProperty("intake_wait_s") // How long the soleniod is enegized for
  public double intakeWait = 0.2; // In seconds

  public IntakeConfig(PneumaticConfig PC) {
    this.PC = PC;
    this.pcmType = PC.pcmType;
  }

  public void update() {
    this.pcmType = PC.pcmType;
    this.simulated = PC.simulated | this.simulated;
  }
}
