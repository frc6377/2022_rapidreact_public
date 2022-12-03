package frc.robot.common.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class GunnerRumble {
  private static XboxController gunner;

  public static void setController(XboxController controller){
    gunner = controller;
  }

  public static void setRumble(double rumble){
    gunner.setRumble(RumbleType.kRightRumble, rumble);
  }
}
