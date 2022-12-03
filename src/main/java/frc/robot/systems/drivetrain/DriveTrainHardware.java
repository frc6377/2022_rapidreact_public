package frc.robot.systems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Constants;
import frc.robot.systems.drivetrain.commands.VelocityDriveCommand;
import frc.robot.common.config.Config.DriverConfig;
import frc.robot.common.motorMath.TankDriveMath;
import frc.robot.common.motorfactory;
import frc.robot.systems.drivetrain.steering.SteeringMix;
import frc.robot.common.utilities.PitchAccessor;
import frc.robot.common.utilities.RollAccessor;

import java.util.function.DoubleSupplier;

public class DriveTrainHardware {
  public WPI_TalonFX driveLeftLead = null;
  public WPI_TalonFX driveRightLead = null;
  public WPI_TalonFX driveLeftFollow = null;
  public WPI_TalonFX driveRightFollow = null;
  public Gyro gyro = null;
  public PitchAccessor pitchAccessor = null;
  public RollAccessor rollAccessor = null;
  public DriveTrainSubsystem DriveTrainSubsystem = null;
  public CommandBase DefaultCommand = null;
  public SteeringMix mixer = null;

  public DriveTrainHardware(XboxController controller, TankDriveConfig cfg, DriverConfig dCfg) {
    // Don't initialize hardware for a simulated drivetrain
    if (cfg.simulated) {
      return;
    }

    if (cfg.isPigeon2) {
      WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(cfg.pigeonID);
      pitchAccessor = new PitchAccessor(pigeon2);
      rollAccessor = new RollAccessor(pigeon2);
      gyro = pigeon2;
    } else {
      WPI_PigeonIMU pigeon = new WPI_PigeonIMU(cfg.pigeonID);
      pitchAccessor = new PitchAccessor(pigeon);
      rollAccessor = new RollAccessor(pigeon);
      gyro = pigeon;
    }
    gyro.reset();
    mixer = new SteeringMix();

    VelocityDriveTrain(controller, cfg, dCfg);
  }

  public void VelocityDriveTrain(
      XboxController controller, TankDriveConfig cfg, DriverConfig dCfg) {
    VelocityDrivetrainMotorConfig(cfg);

    var driveTrain = new VelocitySubsystem(
        driveRightLead,
        driveLeftLead,
        cfg,
        gyro);
    DriveTrainSubsystem = driveTrain;
    DoubleSupplier forward = () -> 0 - controller.getLeftY();
    DoubleSupplier rotation = () -> 0 - controller.getRightX();
    DefaultCommand = new VelocityDriveCommand(forward, rotation, driveTrain, mixer, dCfg);
  }

  public void VelocityDrivetrainMotorConfig(TankDriveConfig cfg) {
    // TODO: Simplify this so we don't need to use arrays like this.
    WPI_TalonFX[] right = configureVelocitySide(cfg, cfg.rightLeaderID, cfg.rightFollowerID, true);
    WPI_TalonFX[] left = configureVelocitySide(cfg, cfg.leftLeaderID, cfg.leftFollowerID, false);
    driveRightLead = right[0];
    driveRightFollow = right[1];
    driveLeftLead = left[0];
    driveLeftFollow = left[1];
  }

  private WPI_TalonFX[] configureVelocitySide(
      TankDriveConfig cfg, int leadID, int followID, boolean isRight) {
    var leadConfig = new motorfactory.MotorConfiguration();
    var followConfig = new motorfactory.MotorConfiguration();

    // set inverts
    if (isRight) {
      rightInvert(cfg, leadConfig);
    } else {
      leftInvert(cfg, leadConfig);
    }
    followConfig.invertType = TalonFXInvertType.FollowMaster;

    // set integrated sensor
    leadConfig.talonFXConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
        .toFeedbackDevice();

    // configure current limits - drivetrains are usually on 40A breakers
    leadConfig.talonFXConfig.supplyCurrLimit = motorfactory.SupplyLimit(40);
    leadConfig.talonFXConfig.statorCurrLimit = motorfactory.CurrentLimit(80);
    followConfig.talonFXConfig.supplyCurrLimit = motorfactory.SupplyLimit(40);
    followConfig.talonFXConfig.statorCurrLimit = motorfactory.CurrentLimit(80);

    // set closed loop parameters
    setDiffVelocityPidConfigs(cfg, leadConfig.talonFXConfig);

    setMotionMagicConfigs(cfg, leadConfig.talonFXConfig);
    // create motors
    WPI_TalonFX leadMotor = motorfactory.createTalonFX(leadID, leadConfig);
    WPI_TalonFX followMotor = motorfactory.createTalonFX(followID, followConfig, leadMotor);

    // make sure they use pid
    leadMotor.selectProfileSlot(Constants.PIDSlotVelocity, Constants.PIDLoopPrimary);
    // reset odometry to 0
    leadMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.CANTimeout);

    // send it up
    WPI_TalonFX[] motors = new WPI_TalonFX[2];
    motors[0] = leadMotor;
    motors[1] = followMotor;
    return motors;
  }

  public void rightInvert(TankDriveConfig cfg, motorfactory.MotorConfiguration motorConfig) {
    TalonFXInvertType invertType;
    if (cfg.physicalLayout.rightInvertIsCounterClockwise) {
      invertType = TalonFXInvertType.CounterClockwise;
    } else {
      invertType = TalonFXInvertType.Clockwise;
    }
    motorConfig.invertType = invertType;
  }

  public void leftInvert(TankDriveConfig cfg, motorfactory.MotorConfiguration motorConfig) {
    TalonFXInvertType invertType;
    if (cfg.physicalLayout.rightInvertIsCounterClockwise) {
      invertType = TalonFXInvertType.Clockwise;
    } else {
      invertType = TalonFXInvertType.CounterClockwise;
    }
    motorConfig.invertType = invertType;
  }

  public void setMotionMagicConfigs(TankDriveConfig cfg, TalonFXConfiguration talonConfig) {
    talonConfig.motionCruiseVelocity = (cfg.controlParameters.velocity * 12)
        / TankDriveMath.distancePerTick(
            cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter);
    talonConfig.motionAcceleration = talonConfig.motionCruiseVelocity / cfg.controlParameters.accelerationTime;
  }

  public void setDiffVelocityPidConfigs(TankDriveConfig cfg, TalonFXConfiguration talonConfig) {
    talonConfig.slot0.kF = TankDriveMath.kF(
        cfg.physicalLayout.gearRatio,
        cfg.physicalLayout.wheelDiameter,
        cfg.physicalLayout.predictedVelocity * 12);
    talonConfig.slot0.kP = TankDriveMath.kP_velocity(
        cfg.physicalLayout.gearRatio,
        cfg.physicalLayout.wheelDiameter,
        cfg.controlParameters.pSaturation * 12);
  }
}
