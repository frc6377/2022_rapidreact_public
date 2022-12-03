package frc.robot.common.config;

import frc.robot.common.config.robots.Mulebot;
import frc.robot.common.config.robots.Omnibot;
import frc.robot.common.config.robots.Quickdraw;
import frc.robot.common.config.robots.Slowdraw;
import frc.robot.common.config.robots.DangerNoodle2;

import java.util.Map;

public class RobotDetector {
  // Map the macaddress of the wifi receiver to a class file.
  // This allows us to determine which robot is being referenced at runtime.
  // TODO: This should be a file on disk so it can be changed if necessary during a match
  private static final Map<String, Class<? extends Config>> robotMap =
      Map.of(
          "0:80:2F:24:36:BF", Slowdraw.class,
          "0:80:2F:16:0D:FF", Omnibot.class,
          "0:80:2F:28:C0:F0", DangerNoodle2.class,
          "0:80:2F:28:94:AD", Quickdraw.class,
          "0:80:2F:33:9E:06", Quickdraw.class,
          //"0:80:2F:24:4B:12", Unknown
          "0:80:2F:22:B3:3E", Mulebot.class
          //"0:80:2F:24:4B:12", Unknown
          );

  public static Class<? extends Config> GetConfigClass(String macAddress) {
    var config = robotMap.get(macAddress);
    if (config == null) {
      return Config.class;
    }

    return config;
  }
}
