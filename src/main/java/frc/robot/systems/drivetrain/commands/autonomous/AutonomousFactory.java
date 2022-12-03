package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSystems;
import frc.robot.common.commands.SimulatedCommand;
import frc.robot.common.config.Config;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;
import java.util.Hashtable;

public class AutonomousFactory {
  private final Hashtable<CommandName, Command> commandMap = new Hashtable<CommandName, Command>();
  private Config cfg;
  private RobotSystems systems;
  private TrajectoryBuilder trajectoryBuilder;

  public AutonomousFactory(RobotSystems systems, Config cfg, TrajectoryBuilder trajectoryBuilder) {
    this.cfg = cfg;
    this.systems = systems;
    this.trajectoryBuilder = trajectoryBuilder;
    commandMap.put(CommandName.Ramsete, new SimulatedCommand());
    commandMap.put(CommandName.ToggleIntake, new SimulatedCommand());
    commandMap.put(CommandName.SpinIntakeRollersIntake, new SimulatedCommand());
    commandMap.put(CommandName.SpinIntakeRollersEject, new SimulatedCommand());
    commandMap.put(CommandName.IdleIntakeRollers, new SimulatedCommand());
    commandMap.put(CommandName.fullAuto, new SimulatedCommand());
    commandMap.put(CommandName.FullAutoOnlyDrive, new SimulatedCommand());
    commandMap.put(CommandName.AutoBlueBallCommandGroup2, new SimulatedCommand());
  }

  public Command get(CommandName cmd) {

    switch (cmd) {
      case fullAuto:
        if (!cfg.tankdriveLayout.simulated
            && !cfg.intakeConfig.simulated
            && !cfg.ballHandlerConfig.simulated
            && !cfg.upgoerConfig.simulated
            && !cfg.turretConfig.simulated
            && !cfg.shooterConfig.simulate
            && !cfg.limelightConfig.simualated) {
          commandMap.put(
              CommandName.fullAuto, new AutonomousRobotCommandGroup(systems, trajectoryBuilder));
        }
        break;

      case FullAutoOnlyDrive:
        if (!cfg.tankdriveLayout.simulated)
          commandMap.put(
              CommandName.FullAutoOnlyDrive, new FullAutoOnlyDrive(systems, trajectoryBuilder));
        break;

      case AutoBlueBallCommandGroup:
        if (!cfg.tankdriveLayout.simulated
            && !cfg.intakeConfig.simulated
            && !cfg.ballHandlerConfig.simulated
            && !cfg.upgoerConfig.simulated
            && !cfg.turretConfig.simulated
            && !cfg.shooterConfig.simulate
            && !cfg.limelightConfig.simualated) {
          commandMap.put(
              CommandName.AutoBlueBallCommandGroup,
              new AutoBlueBallCommandGroup(systems, trajectoryBuilder));
        }
        break;
      case AutoBlueBallCommandGroup2:
        if (!cfg.tankdriveLayout.simulated
            && !cfg.intakeConfig.simulated
            && !cfg.ballHandlerConfig.simulated
            && !cfg.upgoerConfig.simulated
            && !cfg.turretConfig.simulated
            && !cfg.shooterConfig.simulate
            && !cfg.limelightConfig.simualated) {
          commandMap.put(
              CommandName.AutoBlueBallCommandGroup2,
              new AutoBlueBallCommandGroup2(systems, trajectoryBuilder));
        }
        break;
      case FiveBallAuto:
        if (!cfg.tankdriveLayout.simulated
            && !cfg.intakeConfig.simulated
            && !cfg.ballHandlerConfig.simulated
            && !cfg.upgoerConfig.simulated
            && !cfg.turretConfig.simulated
            && !cfg.shooterConfig.simulate
            && !cfg.limelightConfig.simualated) {
          commandMap.put(CommandName.FiveBallAuto, new FiveBallAuto(systems, trajectoryBuilder));
        }
        break;
      case OneBallAuto:
        if (!cfg.tankdriveLayout.simulated
            && !cfg.intakeConfig.simulated
            && !cfg.ballHandlerConfig.simulated
            && !cfg.upgoerConfig.simulated
            && !cfg.turretConfig.simulated
            && !cfg.shooterConfig.simulate
            && !cfg.limelightConfig.simualated) {
          commandMap.put(CommandName.OneBallAuto, new OneBallAuto(systems, trajectoryBuilder));
        }
        break;

      default:
        break;
    }
    System.out.println(cmd);
    return commandMap.get(cmd);
  }

  public enum CommandName {
    Ramsete,
    ToggleIntake,
    SpinIntakeRollersIntake,
    SpinIntakeRollersEject,
    IdleIntakeRollers,
    AutonomousIntake,
    fullAuto,
    TwoBallAuto,
    FullAutoOnlyDrive,
    AutoBlueBallCommandGroup,
    AutoBlueBallCommandGroup2,
    FiveBallAuto,
    OneBallAuto
  }
}
