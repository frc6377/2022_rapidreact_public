package frc.robot.systems.turret;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.utilities.MovingAverage;
import frc.robot.systems.turret.HoodSubsystem.HoodState;

public class LimelightSubsystem extends SubsystemBase {
  private boolean longRange = false;
  private final LimelightConfig cfg;
  private final TargetConfig trgcfg;
  public double distanceToTarget;

  public LimelightSubsystem(LimelightConfig cfg, TargetConfig trgcfg) {
    this.cfg = cfg;
    this.trgcfg = trgcfg;
    distanceToTarget = 0;
  }

  MovingAverage xAverage = new MovingAverage();

  public void periodic() {
  }

  public void setLongRange(boolean isLongRange) {
    longRange = isLongRange;
  }

  public void turnOnLEDs() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void turnOffLEDs() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void turnOnDriverMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  public void turnOffDriverMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }

  /**
   * gets the x offset of the target from network tables
   *
   * @return x offset of the target in degrees
   */
  public double getXOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  /**
   * gets whether or not we have a valid target from network tables
   *
   * @return whether or not we have a valid target
   */
  public boolean hasValidTarget() {
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5)
      return true; // using > 0.5 to avoid errors comparing doubles
    return false;
  }

  /**
   * gets the y offset of the target from network tables
   *
   * @return y offset of the target in degrees
   */
  public double getYOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  /**
   * gets the area of the image that the target takes up from network tables
   *
   * @return area that the target takes up as a %
   */
  public double getTargetArea() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  /**
   * gets the latency of the limelight from network tables
   *
   * @return latency of the limelight in ms
   */
  public double getLatency() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
  }

  /**
   * gets the sidelength of the shortest side of the fitted bounding box from
   * network tables
   *
   * @return the sidelength of the shortest side of the fitted bounding box in
   *         pixels
   */
  public double getShortest() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
  }

  /**
   * gets the sidelength of the longest side of the fitted bounding box from
   * network tables
   *
   * @return the sidelength of the longest side of the fitted bounding box in
   *         pixels
   */
  public double getLongest() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
  }

  /**
   * gets the vertical sidelength of the rough bounding box from network tables
   *
   * @return the vertical sidelength of the rough bounding box in pixels
   */
  public double getVertLen() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
  }

  /**
   * gets the horizontal sidelength of the rough bounding box from network tables
   *
   * @return the horizontal sidelength of the rough bounding box in pixels
   */
  public double getHorLen() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
  }

  /**
   * gets the active pipeline on the limelight from network tables
   *
   * @return the active pipeline on the limelight
   */
  public double getPipe() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0);
  }

  /**
   * computes the distance to the target in inches
   *
   * @return distance to target in inches
   */
  public double getDist() {

    double mountingAngle = cfg.mountingAngleCloseRange;
    double mountingHeight = cfg.mountingHeightCloseRange;

    if (longRange) {
      mountingAngle = cfg.mountingAngleLongRange;
      mountingHeight = cfg.mountingHeightLongRange;
      SmartDashboard.putString("limelight hood pos", "high");
    } else {
      SmartDashboard.putString("limelight hood pos", "low");
    }

    double mountingAngleRadians = Math.toRadians(mountingAngle); // convert degrees to radians
    double targetOffset = Math.toRadians(getYOffset()); // convert degrees to radians

    var distanceToTarget = computeDistance(trgcfg.targetHeight, mountingHeight, mountingAngleRadians, targetOffset);

    SmartDashboard.putNumber("target height", trgcfg.targetHeight);
    SmartDashboard.putNumber("mounting height", mountingHeight);
    SmartDashboard.putNumber("mounting angle", mountingAngle);
    SmartDashboard.putNumber("target offset", targetOffset);

    SmartDashboard.putNumber(
        "distance to target limelight",
        distanceToTarget);
    return distanceToTarget;
  }

  public double computeDistance(double targetHeight, double mountingHeight, double mountingAngle, double targetOffset) {
    /*
     * C in triangle ABC A B
     * solving for AB given BC from knowing the height of the target relative to the
     * camera
     * and knowing angle CAB from the mounting angle of the camera and the Y offset
     * of the vision target
     * take AC and divide it by the tangent of angle CAB to get AB
     */
    return (targetHeight - mountingHeight) / Math.tan(mountingAngle + targetOffset);


  }
}
