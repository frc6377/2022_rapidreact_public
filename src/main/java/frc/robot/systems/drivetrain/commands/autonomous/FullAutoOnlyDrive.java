package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSystems;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;

public class FullAutoOnlyDrive extends SequentialCommandGroup {
    public FullAutoOnlyDrive(RobotSystems systems, TrajectoryBuilder trajBuilder) {
        addCommands(
                new RamseteAutonCommand(
                        "paths/firstBall.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, true),
                new RamseteAutonCommand(
                        "paths/backup.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new RamseteAutonCommand(
                        "paths/2More.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new RamseteAutonCommand(
                        "paths/backupAgain.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false));
    }
}
