package frc.robot.common.config;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;

public class RuntimeConfig {
  private ResourceFile resources;

  public String deployHost;
  public String deployTime;
  public String buildTime;
  public String branch;
  public String commit;
  public String changes;
  public String remote;
  public String macAddress;

  public RuntimeConfig(ResourceFile resources) {
    this.resources = resources;

    deployHost = readFromFile("deployhost.txt", true);
    deployTime = readFromFile("deploytime.txt", true);
    buildTime = readFromFile("buildtime.txt", false);
    branch = readFromFile("branch.txt", false);
    commit = readFromFile("commit.txt", false);
    changes = readFromFile("changes.txt", false);
    remote = readFromFile("remote.txt", false);
    macAddress = identifyRobot();
  }

  private String readFromFile(String filename, boolean isDeploy) {
    InputStream statusfile = null;
    try {
      if (isDeploy) {
        if (RobotBase.isSimulation()) {
          statusfile =
              new BufferedInputStream(
                  new FileInputStream(
                      Filesystem.getLaunchDirectory() + "/src/main/deploy/" + filename));
        } else {
          statusfile =
              new BufferedInputStream(
                  new FileInputStream(Filesystem.getDeployDirectory() + "/" + filename));
        }
      } else {
        statusfile = this.resources.getResource("/" + filename);
      }

      return new String(statusfile.readAllBytes(), StandardCharsets.UTF_8);

    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } finally {
      if (statusfile != null) {
        try {
          statusfile.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }
    return "";
  }

  /**
   * identifyRobot will attempt to find the macaddress of a robot. 
   * It does this by first looking to see if it can find a ethernet address as "eth0".
   * If no ethernet is found, then look to see if usb is plugged in on "usb0".
   * If neither of those is found, then return null.
   * 
   * @return a mac address of one is found for the robot, otherwise null
   */
  private String identifyRobot() {
    try {
      var netInfo = NetworkInterface.getByName("eth0");
      if (netInfo == null) {
         netInfo = NetworkInterface.getByName("usb0");
      }
      if (netInfo == null) {
        return null;
      }
      var hardwareAddress = netInfo.getHardwareAddress();
      StringBuilder sb = new StringBuilder();
      if (hardwareAddress == null) {
        return "simulated";
      }
      for (int i = 0; i < hardwareAddress.length; i++) {
        sb.append(
            String.format(
                "%01X%s", hardwareAddress[i], (i < hardwareAddress.length - 1) ? ":" : ""));
      }
      return sb.toString();
    } catch (SocketException e) {
      // print non-fatal error
      e.printStackTrace();
      return null;
    }
  }

  public interface ResourceFile {
    InputStream getResource(String s);
  }
}
