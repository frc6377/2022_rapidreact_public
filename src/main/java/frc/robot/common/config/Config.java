package frc.robot.common.config;

import com.fasterxml.jackson.annotation.JsonProperty;
import frc.robot.systems.ballhandler.BallHandlerConfig;
import frc.robot.systems.climber.ClimberConfig;
import frc.robot.systems.compressor.PneumaticConfig;
import frc.robot.systems.drivetrain.TankDriveConfig;
import frc.robot.systems.intake.IntakeConfig;
import frc.robot.systems.signaling.SignalingConfig;
import frc.robot.systems.turret.LimelightConfig;
import frc.robot.systems.turret.ShooterConfig;
import frc.robot.systems.turret.TargetConfig;
import frc.robot.systems.turret.TurretConfig;
import frc.robot.systems.ballhandler.UpgoerConfig;

public class Config extends Configable {
  @JsonProperty("robot_name")
  public String robotName = "DefaultBot";

  @JsonProperty("driver_joystick")
  public DriverConfig Driver = new DriverConfig();

  @JsonProperty("gunner_joystick")
  public GunnerConfig Gunner = new GunnerConfig();

  public class DriverConfig {
    @JsonProperty("joystick_port_id")
    public int joystickPortID = 0;

    @JsonProperty("trigger_sensitivity")
    public double triggerSensitivity = 0.2;

    @JsonProperty("high_gear_percentage")
    public double highGearTranslationPercentage = 0.9;

    @JsonProperty("low_gear_percentage")
    public double lowGearTranslationPercentage = 0.7;

    @JsonProperty("high_gear_turn_percent")
    public double highGearMaxTurn = 0.7;

    @JsonProperty("low_gear_turn_percent")
    public double lowGearMaxTurn = 0.5;

    @JsonProperty("deadband")
    public double deadband = 0.1;
  }

  public class GunnerConfig {
    @JsonProperty("joystick_port_id")
    public int joystickPortID = 1;

    @JsonProperty("trigger_sensitivity")
    public double triggerSensitivity = 0.2;
  }

  @JsonProperty("tankdrive_layout")
  public TankDriveConfig tankdriveLayout = new TankDriveConfig();

  @JsonProperty("compressor-ID")
  public int compressorID = 1; // TODO: Actual Value

  @JsonProperty("pneumatic-config")
  public PneumaticConfig pneumaticConfig = new PneumaticConfig();

  @JsonProperty("intake")
  public IntakeConfig intakeConfig = new IntakeConfig(pneumaticConfig);

  @JsonProperty("shooter")
  public ShooterConfig shooterConfig = new ShooterConfig();

  {
    shooterConfig.simulate = false;
    shooterConfig.flywheelID = 9;
    shooterConfig.flywheelFollowID = 11;
    shooterConfig.kf = 0.1;
    shooterConfig.kp = 0;
    shooterConfig.shooterInvertIsClockwise = true;
    shooterConfig.followMasterInvert = false;
    shooterConfig.idleVelo = 1000;
  }

  @JsonProperty("target")
  public TargetConfig targetConfig = new TargetConfig();

  {
    targetConfig.simulated = false;
    targetConfig.targetHeight = 120;
    targetConfig.targetX = 10;
    targetConfig.targetY = 10;
  }

  @JsonProperty("limelight")
  public LimelightConfig limelightConfig = new LimelightConfig();

  {
    limelightConfig.simualated = false;
    limelightConfig.mountingAngleCloseRange = 30;
    limelightConfig.mountingHeightCloseRange = 24;
  }

  @JsonProperty("ball_handler_config")
  public BallHandlerConfig ballHandlerConfig = new BallHandlerConfig();

  @JsonProperty("turret_config")
  public TurretConfig turretConfig = new TurretConfig();

  {
    turretConfig.simulated = false;
    turretConfig.ticsToDegrees = 1 / 150;
    turretConfig.turretId = 30;
    turretConfig.turretRange = 90;
    turretConfig.degreesToTics = 150;
    turretConfig.invertIsClockwise = true;
    turretConfig.kF = 0.051291;
    turretConfig.kP = 0.354916;
    turretConfig.maxAccel = 500;
    turretConfig.maxVelo = 1000;
  }

  @JsonProperty("upgoer_config")
  public UpgoerConfig upgoerConfig = new UpgoerConfig();

  @JsonProperty("climber_config")
  public ClimberConfig climberConfig = new ClimberConfig();

  @JsonProperty("signaling_config")
  public SignalingConfig signalingConfig = new SignalingConfig();
}
