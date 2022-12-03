package frc.robot.systems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Constants;
import frc.robot.common.utilities.RevServo;

public class HoodSubsystem extends SubsystemBase {
  private HoodState hoodState = HoodState.LONG;
  private int servoLockout = 75;

  private final HoodHardware hardware;
  private final ShooterConfig cfg;

  public enum HoodState {
    CLOSE,
    LONG,
    ZERO
  }

  public HoodSubsystem(
      ShooterConfig cfg) {
    this.cfg = cfg;
    this.hardware = new HoodHardware(cfg);

    addChild("left servo", this.hardware.servoLeft);
    addChild("right servo", this.hardware.servoRight);
  }

  public HoodState GetState() {
    return hoodState;
  }

  public void setState(HoodState hoodstate) {
    this.hoodState = hoodstate;
  }

  public void setHoodAngle(HoodState demandHoodState) {
    hoodState = demandHoodState;
    double demandHoodAngle;
    switch (hoodState) {
      case ZERO:
        demandHoodAngle = cfg.hoodPosZero;
        break;

      case CLOSE:
        demandHoodAngle = cfg.hoodPosCloseRange;
        break;

      case LONG:
        demandHoodAngle = cfg.hoodPosLongRange;
        break;

      default:
        demandHoodAngle = cfg.hoodPosLongRange;
        break;
    }
    setAngle(demandHoodAngle);
    SmartDashboard.putNumber("hood target", demandHoodAngle);
  }

  private void setAngle(double angle) {
    hardware.servoRight.setAngle(angle);
    hardware.servoLeft.setAngle(270 - angle);
  }

  // what angle to move hood to from limelight
  public HoodState computeHoodState(
    int servoLockout,
    boolean limelightValidTarget,
    int limelightDist,
    boolean targeting
    ) {
    SmartDashboard.putNumber("servo lockout", servoLockout);

    HoodState newHoodstate = hoodState;
    if (servoLockout < 75) {
      return hoodState;
    }

    if (limelightValidTarget && targeting) {
      switch (hoodState) {
        case CLOSE:
          if (limelightDist > 160) {
            newHoodstate = HoodState.LONG;
          }
          break;

        case LONG:
          if (limelightDist < 100) {
            newHoodstate = HoodState.CLOSE;
          }
          break;

        case ZERO:
          // if zero button accidently hit while playing targeting will
          // break out of it
          if (targeting) { 
            newHoodstate = HoodState.CLOSE;
          } else {
            newHoodstate = HoodState.ZERO;
          }
      }
    }

    else if (hoodState != HoodState.ZERO && targeting) {
      newHoodstate = HoodState.LONG;
    }

    if (newHoodstate != this.hoodState) {
      servoLockout = 0;
    }
    return newHoodstate;
  }

  private class HoodHardware {
    private RevServo servoRight;
    private RevServo servoLeft;

    public HoodHardware(ShooterConfig cfg) {
      servoRight = new RevServo(cfg.hoodRightID);
      servoLeft = new RevServo(cfg.hoodLeftID);
    }

    private int calculateServo(double demand1, double hoodPosHigh) {
      // The goal of this function is to map the range from 0 to 2000
      var hood_position = MathUtil.clamp(demand1, 0, hoodPosHigh) / Constants.Degrees270;
      SmartDashboard.putNumber("hood_position", hood_position);

      var calc_range = (int) Math.round(hood_position * 2000);
      SmartDashboard.putNumber("actual hood target", calc_range);
      return calc_range; // 270 degree range maps onto 2000 microsecond pulse width range
    }
  }
}
