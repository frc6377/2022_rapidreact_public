package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSystems;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;

public class AutonomousRobotCommandGroup extends SequentialCommandGroup {
    public AutonomousRobotCommandGroup(RobotSystems systems, TrajectoryBuilder trajBuilder) {

        addCommands(
                systems.getCommands().toggleIntake(),
                new RamseteAutonCommand(
                        "paths/firstBall.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, true),
                systems.getCommands().idleRollers(),
                new RamseteAutonCommand(
                        "paths/backup.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                systems.getCommands().ballHandlerForward(),
                systems.getCommands().upgoerUp(),
                new WaitCommand(2),
                systems.getCommands().ballHandlerNeutral(),
                systems.getCommands().upgoerNeutral(),
                systems.getCommands().toggleIntake(),
                new RamseteAutonCommand(
                        "paths/2More.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new RamseteAutonCommand(
                        "paths/backupAgain.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                systems.getCommands().ballHandlerForward(),
                systems.getCommands().upgoerUp());
    }
}
