package frc.robot.systems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Constants;
import frc.robot.common.motorMath.TankDriveMath;

public class VelocitySubsystem extends SubsystemBase implements DriveTrainSubsystem {
  public WPI_TalonFX driveLeft;
  public WPI_TalonFX driveRight;
  private final TankDriveConfig cfg;
  private final Field2d field = new Field2d();
  private int trajNumber = 0; // helps all trajectories get in glass

  // Odometry class for tracking robot pose where the X,Y coordinate system of the
  // field is measured
  // in inches
  private final DifferentialDriveOdometry odometry;
  private Gyro gyro;

  public VelocitySubsystem(
      WPI_TalonFX driveRight,
      WPI_TalonFX driveLeft,
      TankDriveConfig cfg,
      Gyro gyro) {
    this(
        driveRight,
        driveLeft,
        cfg,
        gyro,
        new DifferentialDriveOdometry(new Rotation2d()));

    SmartDashboard.putData(field);
  }

  public VelocitySubsystem(
      WPI_TalonFX driveRight,
      WPI_TalonFX driveLeft,
      TankDriveConfig cfg,
      Gyro gyro,
      DifferentialDriveOdometry odometry)  {
    this.driveRight = driveRight;
    this.driveLeft = driveLeft;
    this.cfg = cfg;
    this.odometry = odometry;
    this.gyro = gyro;

    // TODO set the initial robot odonmetry Pose
    addChild("driveLeft", this.driveLeft);
    addChild("driveRight", this.driveRight);
    addChild("Gyro", (Sendable) this.gyro);
    addChild("Field", field);
  }

  @Override
  public void periodic() {
    // Update the odometry using information from the Pigeon and the motors
    // Motor encoders are in raw ticks which need to be converted to inches

    var rotation = new Rotation2d(gyro.getRotation2d().getRadians());
    odometry.update(
        rotation,
        TankDriveMath.distancePerTick(
            cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
            * driveRight.getSelectedSensorPosition(0) // TODO: Right and left are inverted here.
            / Constants.MetersToInches,
        TankDriveMath.distancePerTick(
            cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
            * driveLeft.getSelectedSensorPosition(0)
            / Constants.MetersToInches);
    field.setRobotPose(odometry.getPoseMeters());
  }

  public void Drive(DifferentialDrive.WheelSpeeds driveWheels) {
    double velocity = cfg.controlParameters.velocity;

    double leftVelocity = driveWheels.left
        * (velocity
            * 12
            / (TankDriveMath.distancePerTick(
                cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
                * 10)); 
    double rightVelocity = driveWheels.right
        * (velocity
            * 12
            / (TankDriveMath.distancePerTick(
                cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
                * 10));

    driveLeft.set(TalonFXControlMode.Velocity, leftVelocity);
    driveRight.set(TalonFXControlMode.Velocity, rightVelocity);
    SmartDashboard.putNumber("left speed target", leftVelocity);
    SmartDashboard.putNumber("right speed target", rightVelocity);
  }

  public void sendTrajToTables(Trajectory traj) {
    field.getObject("traj" + trajNumber).setTrajectory(traj);
    trajNumber++;
  }

  @Override
  public void drive(Double left, Double right) {
    this.Drive(new DifferentialDrive.WheelSpeeds(left, right));
  }

  @Override
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  private void resetEncoders() {
    driveLeft.setSelectedSensorPosition(0.0);
    driveRight.setSelectedSensorPosition(0.0);
  }

  @Override
  public void driveAuto(double right, double left) {

    double velocity = cfg.controlParameters.velocity;

    double leftVelocity = left
        * Constants.MetersToInches
        / (TankDriveMath.distancePerTick(
            cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
            * 10);
    double rightVelocity = right
        * Constants.MetersToInches
        / (TankDriveMath.distancePerTick(
            cfg.physicalLayout.gearRatio, cfg.physicalLayout.wheelDiameter)
            * 10);
    driveLeft.set(TalonFXControlMode.Velocity, leftVelocity);
    driveRight.set(TalonFXControlMode.Velocity, rightVelocity);
  }

  public void updateOdomFromLimelight(
      double deltaDistance, double turretHeading, double targetX, double targetY) {
    double deltaX = Math.sin(turretHeading - odometry.getPoseMeters().getRotation().getDegrees())
        * deltaDistance;
    double deltaY = Math.cos(turretHeading - odometry.getPoseMeters().getRotation().getDegrees())
        * deltaDistance;
    odometry.update(odometry.getPoseMeters().getRotation(), targetX + deltaX, targetY + deltaY);
  }
}
