package frc.robot.systems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* Placeholder. Common drive train methods and properties go here */
public interface DriveTrainSubsystem extends Subsystem {

  public void drive(Double x, Double y);

  public Pose2d getPose();

  public void resetOdometry(Pose2d initialPose);

  public void driveAuto(double x, double y);

  public void sendTrajToTables(Trajectory trajectory);

  public void updateOdomFromLimelight(double w, double x, double y, double z);
}
