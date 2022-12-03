package frc.robot.systems.drivetrain.steering;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.common.Constants;
import frc.robot.common.config.Config.DriverConfig;

public class SteeringMix {
  protected static boolean lowGear = true;

  public DifferentialDrive.WheelSpeeds joystickToTank(Double xSpeed, Double zRotation, DriverConfig dCfg) {
    xSpeed = MathUtil.clamp(xSpeed, -1, 1);
    xSpeed = MathUtil.applyDeadband(xSpeed, dCfg.deadband);
    xSpeed = xSpeed * ((lowGear) ? dCfg.lowGearTranslationPercentage : dCfg.highGearTranslationPercentage);

    zRotation = MathUtil.clamp(zRotation, -1, 1);
    zRotation = MathUtil.applyDeadband(zRotation, dCfg.deadband);
    zRotation = zRotation * ((lowGear) ? dCfg.lowGearTranslationPercentage : dCfg.highGearTranslationPercentage);
    zRotation *= lowGear ? dCfg.lowGearMaxTurn : dCfg.highGearMaxTurn;

    var turnInPlace = Math.abs(xSpeed) < Constants.doubleEpsilon;

    // TODO: Erik here is our test drive.
    // return DifferentialDrive.curvatureDriveIK(xSpeed, zRotation, turnInPlace);
    return mix(xSpeed, zRotation);
  }

  public static void changeGear(boolean islowGear) {
    lowGear = islowGear;
  }

    /**
   * This function is meant to look like the DifferentialDrive mixer called
   * "curvatureDriveIK". This mixer differs because it looks at each quadrant
   * rather than doing a magnitude calculation at the end.
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The normalized curvature [-1.0..1.0]. Clockwise is positive.
   * @return Wheel speeds [-1.0..1.0].
   */
  private DifferentialDrive.WheelSpeeds mix(double xSpeed, double zRotation) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    double leftSpeed;
    double rightSpeed;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotation;
      } else {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotation;
      }
    }

    return new DifferentialDrive.WheelSpeeds(leftSpeed, rightSpeed);
  }
}
