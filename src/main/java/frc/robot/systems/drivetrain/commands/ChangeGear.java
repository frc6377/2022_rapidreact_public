package frc.robot.systems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.drivetrain.steering.SteeringMix;

public class ChangeGear extends InstantCommand {
  private boolean lowGear;

  public ChangeGear(boolean islowGear) {
    lowGear = islowGear;
  }
  
  @Override
  public void initialize() {
    SteeringMix.changeGear(lowGear);
  }
}
