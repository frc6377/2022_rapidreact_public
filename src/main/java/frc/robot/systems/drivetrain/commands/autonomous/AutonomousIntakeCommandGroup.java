package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSystems;
import frc.robot.systems.intake.IntakeConfig;

public class AutonomousIntakeCommandGroup extends SequentialCommandGroup {
    public AutonomousIntakeCommandGroup(RobotSystems systems, IntakeConfig cfg) {
        addCommands(
                systems.getCommands().toggleIntake(),
                new WaitCommand(cfg.intakeWait),
                new InstantCommand(() -> systems.Intake.neutralExtension()),
                systems.getCommands().intakeCargo(),
                new WaitCommand(5.0),
                systems.getCommands().idleRollers(),
                systems.getCommands().toggleIntake(),
                new WaitCommand(cfg.intakeWait),
                new InstantCommand(() -> systems.Intake.neutralExtension()));
    }
}
