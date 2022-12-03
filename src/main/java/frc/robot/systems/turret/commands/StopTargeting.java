package frc.robot.systems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotSystems;

public class StopTargeting extends ParallelCommandGroup {

    public StopTargeting(RobotSystems systems) {
        addCommands(
                systems.getCommands().turnOffLEDs(),
                systems.getCommands().pointAtHub(),
                systems.getCommands().shooterIdle());
    }
}
