package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSystems;
import frc.robot.systems.turret.commands.Target;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;

public class AutoBlueBallCommandGroup extends SequentialCommandGroup {

    public AutoBlueBallCommandGroup(RobotSystems systems, TrajectoryBuilder trajectorybuilder) {
        addCommands(
                new InstantCommand(() -> systems.Turret.setRobotNorth(90)),
                systems.getCommands().toggleIntake(),
                systems.getCommands().intakeCargo(),
                systems.getCommands().ballHandlerForward(),
                new Target(systems),
                new RamseteAutonCommand(
                        "paths/blueballpath.wpilib.json",
                        systems.DriveTrain.DriveTrainSubsystem,
                        trajectorybuilder,
                        true),
                new WaitCommand(1),
                systems.getCommands().toggleIntake(),
                systems.getCommands().upgoerUp());
    }
}
