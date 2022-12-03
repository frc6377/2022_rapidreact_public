package frc.robot.systems.drivetrain.steering;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.systems.drivetrain.TankDriveConfig;
import java.util.List;

public class TrajectoryBuilder {
  public final DifferentialDriveKinematics differentialDriveKinematics;
  public final TrajectoryConfig trajectoryConfig;
  public final Trajectory trajectory;

  public TrajectoryBuilder(TankDriveConfig config) {
    differentialDriveKinematics = new DifferentialDriveKinematics(config.physicalLayout.wheelBase);

    // Create config for trajectory
    trajectoryConfig =
        new TrajectoryConfig(
                config.controlParameters.velocity, config.controlParameters.accelerationTime)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(differentialDriveKinematics);

    // An example trajectory to follow.  All units in inches (dervied from the base example which
    // uses meters).
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(30, 30), new Translation2d(60, -30)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(90, 0, new Rotation2d(0)),
            // Pass config
            trajectoryConfig);
  }
}
