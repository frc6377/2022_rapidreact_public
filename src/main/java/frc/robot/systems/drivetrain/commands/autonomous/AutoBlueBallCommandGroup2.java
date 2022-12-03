package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSystems;
import frc.robot.systems.turret.commands.Target;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;

public class AutoBlueBallCommandGroup2 extends SequentialCommandGroup {

    public AutoBlueBallCommandGroup2(RobotSystems systems, TrajectoryBuilder trajectoryBuilder) {
        CommandGroupBase.clearGroupedCommands();
        addCommands(
                systems.getCommands().toggleIntake(),
                systems.getCommands().intakeCargo(),
                systems.getCommands().ballHandlerForward(),
                new Target(systems),
                new RamseteAutonCommand(
                        "paths/anotherBluePath.wpilib.json",
                        systems.DriveTrain.DriveTrainSubsystem,
                        trajectoryBuilder,
                        true),
                systems.getCommands().toggleIntake(),
                new WaitCommand(1),
                systems.getCommands().upgoerUp());
    }
}
