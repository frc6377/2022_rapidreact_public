package frc.robot.systems.ballhandler;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.motorfactory;

public class BallHandlerSubsystem extends SubsystemBase {
    private final BallHandlerConfig config;
    private final BallHandlerHardware hardware;

    public BallHandlerSubsystem(BallHandlerConfig config) {
        this.config = config;
        this.hardware = new BallHandlerHardware(config);
        addChild("ball handler", hardware.internalHandlingFront);
    }

    public void setForward() {
        updateMotor(config.forwardVelocity);
    }

    public void setReverse() {
        updateMotor(config.reverseVelocity);
    }

    public void setNeutral() {
        updateMotor(0.0);
    }

    private void updateMotor(double velocity) {
        hardware.internalHandlingFront.set(TalonFXControlMode.PercentOutput, velocity);
        SmartDashboard.putNumber("ballhander velocity", velocity);
    }

    public class BallHandlerHardware {
        public WPI_TalonFX internalHandlingFront;

        public BallHandlerHardware(BallHandlerConfig cfg) {
            if (cfg.simulated) {
                return;
            }

            var internalHandlingFrontConfig = new motorfactory.MotorConfiguration();
            internalHandlingFrontConfig.talonFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 1);
            internalHandlingFrontConfig.talonFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 1, 1, 1);

            internalHandlingFront =
                    motorfactory.createTalonFX(cfg.frontFlywheelsBeltMotorID, internalHandlingFrontConfig);
        }
    }
}