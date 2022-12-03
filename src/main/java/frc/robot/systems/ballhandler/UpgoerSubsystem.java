package frc.robot.systems.ballhandler;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpgoerSubsystem extends SubsystemBase {
  private final UpgoerConfig cfg;
  private final UpgoerHardware hdw;

  public enum State {
    UP,
    DOWN,
    NEUTRAL
  }

  public UpgoerSubsystem(UpgoerConfig cfg) {
    this.cfg = cfg;
    this.hdw = new UpgoerHardware(cfg);
  }

  public void RunUpgoer(State state) {
    double runVelocity = 0;
    if (state == State.UP) {
      runVelocity = cfg.upOutput;
    }
    else if (state == State.DOWN) {
      runVelocity = cfg.downOutput;
    }
    else {
      runVelocity = 0;
    }

    hdw.upgoer.set(TalonFXControlMode.PercentOutput, runVelocity);
  }
  public class UpgoerHardware {
    public UpgoerSubsystem upgoerSubsystem = null;
    public TalonFX upgoer = null;

    public UpgoerHardware(UpgoerConfig cfg) {
      if (!cfg.simulated) {
        upgoer = new TalonFX(cfg.upgoerLeftMotorID);
        upgoer.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 1, 1, 1));
        upgoer.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 1));
      }
    }
  }
}