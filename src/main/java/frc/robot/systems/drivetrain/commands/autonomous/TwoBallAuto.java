package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSystems;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(RobotSystems systems, TrajectoryBuilder trajBuilder) {
        addCommands(
                systems.getCommands().intakeCargo(),
                new RamseteAutonCommand(
                        "paths/firstBall.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, true),
                systems.getCommands().ballHandlerForward(),
                systems.getCommands().upgoerUp());
    }
}
