package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSystems;
import frc.robot.systems.turret.commands.Target;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;

public class FiveBallAuto extends SequentialCommandGroup {
    public FiveBallAuto(RobotSystems systems, TrajectoryBuilder trajBuilder) {
        addCommands(
                systems.getCommands().toggleIntake(),
                systems.getCommands().intakeCargo(),
                systems.getCommands().ballHandlerForward(),
                new Target(systems),
                new RamseteAutonCommand("paths/5BallAuto/firstBall.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, true),
                new RamseteAutonCommand("paths/5BallAuto/backup.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new WaitCommand(0.1),
                systems.getCommands().toggleIntake(),
                systems.getCommands().upgoerUp(),
                new WaitCommand(1),
                systems.getCommands().toggleIntake(),
                systems.getCommands().upgoerNeutral(),
                systems.getCommands().intakeCargo(),
                new RamseteAutonCommand("paths/5BallAuto/1More.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new WaitCommand(0.1),
                systems.getCommands().toggleIntake(),
                systems.getCommands().upgoerUp(),
                new WaitCommand(1),
                systems.getCommands().intakeCargo(),
                systems.getCommands().upgoerNeutral(),
                new RamseteAutonCommand("paths/5BallAuto/2More.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new WaitCommand(0.1),
                new RamseteAutonCommand("paths/5BallAuto/backupAgain.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, false),
                new WaitCommand(0.1),
                systems.getCommands().intakeCargo(),
                systems.getCommands().toggleIntake(),
                systems.getCommands().upgoerUp());
    }
}
