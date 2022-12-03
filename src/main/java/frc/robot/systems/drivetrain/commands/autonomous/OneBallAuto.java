package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotSystems;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;
import frc.robot.systems.turret.commands.Target;

public class OneBallAuto extends SequentialCommandGroup {
    public OneBallAuto(RobotSystems systems, TrajectoryBuilder trajBuilder) {
        addCommands(
                new InstantCommand(() -> systems.Turret.setRobotNorth(0)),
                new Target(systems),
                new RamseteAutonCommand(
                        "paths/1Ball.json", systems.DriveTrain.DriveTrainSubsystem, trajBuilder, true),
                new WaitCommand(1),
                systems.getCommands().upgoerUp(),
                systems.getCommands().ballHandlerForward());
    }
}
