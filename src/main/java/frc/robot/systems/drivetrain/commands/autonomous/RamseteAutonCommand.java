package frc.robot.systems.drivetrain.commands.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;
import frc.robot.systems.drivetrain.DriveTrainSubsystem;
import java.io.IOException;
import java.nio.file.Path;

public class RamseteAutonCommand extends SequentialCommandGroup {

  public RamseteAutonCommand(
      String pathOfTrajToRun,
      DriveTrainSubsystem driveSubsystem,
      TrajectoryBuilder trajectoryBuilder,
      boolean isFirstPath) {
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathOfTrajToRun);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError(
          "Unable to open trajectory: " + pathOfTrajToRun, ex.getStackTrace());
    }
    driveSubsystem.sendTrajToTables(trajectory);

    var ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveSubsystem::getPose,
            new RamseteController(), // using default constuctor with beta/zeta of 2.0 and 0.7
            // because these are recommended drivetrain agnostic values and
            // we dont want to change them
            trajectoryBuilder.differentialDriveKinematics,
            // RamseteCommand passes left and right wheel speeds to the callback
            driveSubsystem::driveAuto,
            driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    Pose2d initialPose =
        new Pose2d(
            trajectory.getInitialPose().getTranslation(),
            new Rotation2d(trajectory.getInitialPose().getRotation().getRadians() - 0));

    if (isFirstPath) driveSubsystem.resetOdometry(initialPose);

    // Run path following command, then stop at the end.
    addCommands(ramseteCommand, new InstantCommand(() -> driveSubsystem.drive(0.0, 0.0)));
  }
}
