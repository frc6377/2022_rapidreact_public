package frc.robot.common.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * TriggerSupplier is a wrapper to make it easier to turn a xbox controller button into a trigger.
 */
public class TriggerSupplier {
  public static Trigger getAButton(XboxController controller) {
    return new JoystickButton(controller, XboxController.Button.kA.value);
  }

  public static Trigger getXButton(XboxController controller) {
    return new JoystickButton(controller, XboxController.Button.kX.value);
  }

  public static Trigger getBButton(XboxController controller) {
    return new JoystickButton(controller, XboxController.Button.kB.value);
  }

  public static Trigger getYButton(XboxController controller) {
    return new JoystickButton(controller, XboxController.Button.kY.value);
  }

  public static Trigger getRightBumper(XboxController controller) {
    return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  }

  public static Trigger getLeftBumper(XboxController controller) {
    return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  }

  public static Trigger getRightTrigger(XboxController controller, double sensitity) {
    return new Trigger(() -> controller.getRightTriggerAxis() >= sensitity);
  }

  public static Trigger getLeftTrigger(XboxController controller, double sensitity) {
    return new Trigger(() -> controller.getLeftTriggerAxis() >= sensitity);
  }

  public static Trigger upDPAd(XboxController controller) {
    return new Trigger(() -> controller.getPOV() == 0);
  }

  public static Trigger rightDPAd(XboxController controller) {
    return new Trigger(() -> controller.getPOV() == 90);
  }

  public static Trigger downDPAd(XboxController controller) {
    return new Trigger(() -> controller.getPOV() == 180);
  }

  public static Trigger leftDPAd(XboxController controller) {
    return new Trigger(() -> controller.getPOV() == 270);
  }

  public static Trigger getBackButton(XboxController controller){
    return new JoystickButton(controller, XboxController.Button.kBack.value);
  }

  public static Trigger getStartsButton(XboxController controller){
    return new JoystickButton(controller, XboxController.Button.kStart.value);
  }
}
