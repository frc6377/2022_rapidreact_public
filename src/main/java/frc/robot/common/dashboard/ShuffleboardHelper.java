package frc.robot.common.dashboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.config.Config;
import frc.robot.common.config.RuntimeConfig;

import java.util.HashMap;
import java.util.Map;

public final class ShuffleboardHelper extends SubsystemBase {
  public static final String DebugTabName = "Debug";
  public static final String DistrictsTabName = "Districts";
  private static Map<String, SimpleWidget> simpleWidgets = new HashMap<>();

  private static ShuffleboardTab comp = Shuffleboard.getTab("Competition Mode");
  private static ShuffleboardTab districts;

  private static ShuffleboardHelper instance = null;

  public ShuffleboardHelper() {
    // Exists only to defeat instantiation.
  }

  public static ShuffleboardHelper getInstance() {
    if (instance == null) {
      instance = new ShuffleboardHelper();
    }
    return instance;
  }

  public static ShuffleboardTab DebugTab() {
    return Shuffleboard.getTab(DebugTabName);
  }

  public static ShuffleboardTab DistrictsTab() {
    districts = Shuffleboard.getTab(DistrictsTabName);
    // districts.addCamera("Driver Station Camera", "6377DangerNoodle",
    // "10.63.77.12:8081");
    return districts;
  }

  public NetworkTableEntry location = DistrictsTab().add("Location", DriverStation.getLocation()).getEntry();
  public NetworkTableEntry matchTime = DistrictsTab().add("Match Time", (int) Math.floor(DriverStation.getMatchTime()))
      .getEntry();

  public static void AddRuntimeConfig(RuntimeConfig runCfg, Config cfg) {
    var layout = ShuffleboardHelper.DebugTab()
        .getLayout("Runtime", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    layout.add("runtime_robotname", "Robot: " + cfg.robotName);
    layout.add("runtime_macaddress", "Mac Address: " + runCfg.macAddress);
    layout.add("runtime_deploytime", "Deploy Time: " + runCfg.deployTime);
    layout.add("runtime_buildtime", "Build Time: " + runCfg.buildTime);
    layout.add("runtime_gitbranch", "Git Branch: " + runCfg.branch);
  }

  public static SimpleWidget putValueComp(String key, Object value) {
    SimpleWidget widget = comp.add(key, value);
    simpleWidgets.put(key, widget);
    return widget;
  }

  public static SimpleWidget putValueDistricts(String key, Object value) {
    SimpleWidget widget = ShuffleboardHelper.DistrictsTab().add(key, value);
    simpleWidgets.put(key, widget);
    return widget;
  }

  public static double getNumberComp(String key, double defaultValue) {
    return simpleWidgets.get(key).getEntry().getDouble(defaultValue);
  }

  public static void addFMSDataDistricts() {
    DistrictsTab().add("Game Data", DriverStation.getGameSpecificMessage().toUpperCase());
    DistrictsTab().add("Alliance Color", DriverStation.getAlliance().toString());
  }

  public void updateFMSDataDistricts() {
    matchTime.setDouble((int) Math.floor(DriverStation.getMatchTime()));
    location.setDouble(DriverStation.getLocation());
  }

  public static void putValueDebug(String key, Object value) {
    Shuffleboard.getTab("debug").add(key, value);
  }

  int i = 0;

  public static void initialize() {
    DebugTab();
    DistrictsTab();
    addFMSDataDistricts();
  }

  @Override
  public void periodic() {
    if (i >= 5) {
      i = 0;
      updateFMSDataDistricts();
    }

    i += 1;
  }
}
