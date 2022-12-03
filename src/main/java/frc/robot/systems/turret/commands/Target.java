package frc.robot.systems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotSystems;

public class Target extends ParallelCommandGroup {
    public Target(RobotSystems systems) {
        addCommands(
                systems.getCommands().turnOnLEDs(),
                systems.getCommands().turretTarget(),
                systems.getCommands().shooterTarget());
    }
}
