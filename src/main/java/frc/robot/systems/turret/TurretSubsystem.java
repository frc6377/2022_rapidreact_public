package frc.robot.systems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MotorGetter;
import frc.robot.common.MotorSetter;
import frc.robot.common.PoseGetter;
import frc.robot.common.motorfactory;
import frc.robot.common.utilities.MovingAverage;

public class TurretSubsystem extends SubsystemBase {
  private final LimelightSubsystem limelight;
  private final TurretConfig cfg;
  private final TargetConfig trgcfg;
  private final PoseGetter poseGetter;
  private Pose2d pose;
  private TurretState turretState;
  private double robotNorth = -90;
  private double autoOffset = 0;
  private MovingAverage dataSet = new MovingAverage();
  private final TurretHardware hardware;

  public TurretSubsystem(
      LimelightSubsystem limelight,
      TurretConfig cfg,
      TargetConfig trgcfg,
      PoseGetter pigeonGetter) {
    this.limelight = limelight;
    this.cfg = cfg;
    this.trgcfg = trgcfg;
    this.poseGetter = pigeonGetter;
    this.hardware = new TurretHardware(cfg);
    turretState = TurretState.IDLE;
  }

  public enum TurretState {
    TARGETING,
    IDLE
  }

  // many ways the math could be handled, args subject to change
  public void pointAtHub() {
    setTurretAbsolute(findHubAngle()); // must be normal set turret for QD
  }

  public void pointAtVisionTarget() {
    setTurret(-limelight.getXOffset() + autoOffset + offset());
  }

  public void pointAtVisionTargetWithRejection() {
    double rawDemand = -limelight.getXOffset();
    dataSet.addData(rawDemand);
    if (!dataSet.isOutlier(rawDemand)) {
      pointAtVisionTarget();
    }
  }

  private double findHubAngle() {
    double robotX = pose.getX();
    double robotY = pose.getY();
    double robotRotation = pose.getRotation().getRadians() + Math.toRadians(180);
    double temp = robotRotation + Math.atan((robotX - trgcfg.targetX) / (robotY - trgcfg.targetY));
    temp = temp * (180 / Math.PI);
    SmartDashboard.putNumber("heading to target", temp + 90);
    return temp + 90; // magic number, idk why it works
  }

  public void setTurretAbsolute(double pos) {
    hardware.turret.set(ControlMode.MotionMagic, pos * cfg.degreesToTics);
  }

  public void setTurret(double pos) {
    var sensorPosition = hardware.turret.getSelectedSensorPosition();
    double setpoint = sensorPosition + (pos * cfg.degreesToTics);
    SmartDashboard.putNumber("tsetpoint preclamp", setpoint);
    setpoint = MathUtil.clamp(setpoint, -(cfg.turretRange*cfg.degreesToTics), cfg.turretRange*cfg.degreesToTics);
    SmartDashboard.putNumber("tsetpoint postclamp", setpoint);
    hardware.turret.set(ControlMode.MotionMagic, setpoint);
  }

  public boolean canSeeTarget() {
    return limelight
        .hasValidTarget(); // could also pass limelight object into command but i opted for an
    // accessor cause it was easier
  }

  @Override
  public void periodic() {
    pose = poseGetter.get();
    String temp = "error";
    if (turretState == TurretState.IDLE) temp = "idle";
    else if (turretState == TurretState.TARGETING) temp = "targeting";
    SmartDashboard.putString("turret state", temp);
    SmartDashboard.putNumber("turret angle", getTurretHeading());
    SmartDashboard.putNumber("heading to hub", getTurretHeading() - limelight.getXOffset());
    setAutoOffset(SmartDashboard.getNumber("turret offset", 0));
    if (limelight.hasValidTarget() && turretState == TurretState.TARGETING) {
      pointAtVisionTarget();
    }else if (turretState != TurretState.TARGETING){
      setTurretAbsolute(robotNorth);
      dataSet.addData(0);
    }
  }

  public double getTurretHeading() {

    return hardware.turret.getSelectedSensorPosition() * cfg.ticsToDegrees;
  }

  public double getTargetX() {
    return trgcfg.targetX;
  }

  public double getTargetY() {
    return trgcfg.targetY;
  }

  public void setTurretState(TurretState newTurretState) {
    turretState = newTurretState;
  }

  public void setRobotNorth(double robotNorth) {
    this.robotNorth = robotNorth;
  }

  public void setAutoOffset (double autoOffset) {
    this.autoOffset = autoOffset;
  }

  public double offset(){
    return ((limelight.getDist()*-0.0625) + 7);
  }

  private class TurretHardware {
    public TalonFX turret;

    public TurretHardware(
            TurretConfig cfg) {
      if (cfg.simulated) return;
      var turretConfig = new motorfactory.MotorConfiguration();

      invert(cfg, turretConfig);
      setClosedLoopGains(cfg, turretConfig);
      turretConfig.neutralMode = NeutralMode.Brake;
      turretConfig.talonFXConfig.forwardSoftLimitEnable = true;
      turretConfig.talonFXConfig.reverseSoftLimitEnable = true;
      turretConfig.talonFXConfig.forwardSoftLimitThreshold =
              (cfg.turretRange / 2) * cfg.degreesToTics;
      turretConfig.talonFXConfig.reverseSoftLimitThreshold =
              -(cfg.turretRange / 2) * cfg.degreesToTics;
      turretConfig.talonFXConfig.primaryPID.selectedFeedbackSensor =
              TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
      turretConfig.talonFXConfig.motionAcceleration = cfg.maxAccel;
      turretConfig.talonFXConfig.motionCruiseVelocity = cfg.maxVelo;

      turret = motorfactory.createTalonFX(cfg.turretId, turretConfig);
    }

    private void invert(TurretConfig cfg, motorfactory.MotorConfiguration motorConfig) {
      motorConfig.invertType = TalonFXInvertType.CounterClockwise;
      if (cfg.invertIsClockwise) motorConfig.invertType = TalonFXInvertType.Clockwise;
    }

    private void setClosedLoopGains(TurretConfig cfg, motorfactory.MotorConfiguration motorConfig) {
      motorConfig.talonFXConfig.slot0.kF = cfg.kF;
      motorConfig.talonFXConfig.slot0.kP = cfg.kP;
    }
  }
}
