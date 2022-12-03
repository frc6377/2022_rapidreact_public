package frc.robot.common;

import frc.robot.common.Constants;
import frc.robot.common.config.Config;
import frc.robot.common.config.RuntimeConfig;

public final class Splash {

  public static boolean printAllStatusFiles(Config cfg, RuntimeConfig runCfg) {

    // Print the Splash Screen
    System.out.println("====================================================================");
    System.out.println("====================================================================");
    System.out.println("====================================================================");
    System.out.println("====================================================================");
    System.out.println("Starting robotInit for Howdy bots " + Constants.TeamNumber);
    System.out.println("Robot Name:" + cfg.robotName);
    System.out.println("Deploy Host: " + runCfg.deployHost);
    System.out.println("Deploy Time: " + runCfg.deployTime);
    System.out.println("Build Time: " + runCfg.buildTime);
    System.out.println("Branch: " + runCfg.branch);
    System.out.println("Commit: " + runCfg.commit);
    System.out.println("Changes: " + runCfg.changes);
    System.out.println("Remote: " + runCfg.remote);
    System.out.println("MAC Address: " + runCfg.macAddress);
    System.out.println("====================================================================");

    return true;
  }
}
