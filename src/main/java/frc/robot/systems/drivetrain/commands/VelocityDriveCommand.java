package frc.robot.systems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.config.Config.DriverConfig;
import frc.robot.systems.drivetrain.steering.SteeringMix;
import frc.robot.systems.drivetrain.VelocitySubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class VelocityDriveCommand extends CommandBase {
  private final VelocitySubsystem subsystem;
  private final DoubleSupplier forward;
  private final DoubleSupplier rotation;
  private final DriverConfig dCfg;
  private final SteeringMix mixer;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public VelocityDriveCommand(
      DoubleSupplier forward,
      DoubleSupplier rotation,
      VelocitySubsystem subsystem,
      SteeringMix mixer,
      DriverConfig dCfg) {
    this.subsystem = subsystem;
    this.forward = forward;
    this.rotation = rotation;
    this.dCfg = dCfg;
    this.mixer = mixer;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    var drive_wheels = mixer.joystickToTank(forward.getAsDouble(), rotation.getAsDouble(), dCfg);
    subsystem.Drive(drive_wheels);
  }
}
