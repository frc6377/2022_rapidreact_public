package frc.robot.systems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.motorfactory;

public class IntakeSubsystem extends SubsystemBase {
    private boolean extended = true;
    private final WPI_TalonFX roller;
    private final DoubleSolenoid solenoid;
    private final IntakeConfig cfg;

    public IntakeSubsystem(IntakeConfig cfg) {
        IntakeHardware hardware = new IntakeHardware(cfg);
        roller = hardware.roller;
        solenoid = hardware.solenoid;
        this.cfg = cfg;

        addChild("roller", roller);
        addChild("Value", solenoid);
    }

    public void toggleIntake() {
        if (extended) {
            retract();
        } else {
            extend();
        }
    }

    public void extend() {
        if (extended) return;
        solenoid.set(DoubleSolenoid.Value.kForward);
        extended = !extended;
    }

    public void retract() {
        if (!extended) return;
        solenoid.set(DoubleSolenoid.Value.kReverse);
        extended = !extended;
    }

    public void neutralExtension() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void intakeCargo() {
        roller.set(ControlMode.PercentOutput, cfg.intakePercentage);
    }

    public void ejectCargo() {
        roller.set(ControlMode.PercentOutput, cfg.ejectPercentage);
    }

    public void idleRollers() {
        roller.set(ControlMode.PercentOutput, 0);
    }

    public class IntakeHardware {
        public final WPI_TalonFX roller;
        public final DoubleSolenoid solenoid;

        /**
         * Creates a IntakeHardware, in accordance to the config given.
         *
         * @param cfg - configuration to create intake hardware in
         */
        public IntakeHardware(IntakeConfig cfg) {
            if (cfg.simulated) {
                roller = null;
                solenoid = null;
                return;
            }
            motorfactory.MotorConfiguration intakeConfig = new motorfactory.MotorConfiguration();
            intakeConfig.talonFXConfig.statorCurrLimit =
                    new StatorCurrentLimitConfiguration(false, 1, 1, 1);
            intakeConfig.talonFXConfig.supplyCurrLimit =
                    new SupplyCurrentLimitConfiguration(false, 1, 1, 1);
            roller = motorfactory.createTalonFX(cfg.rollerId, intakeConfig);
            solenoid =
                    new DoubleSolenoid(
                            cfg.pneumaticControllerID,
                            cfg.pcmType,
                            cfg.soleniodExtensionModuleId,
                            cfg.soleniodRetrationModuleId);
        }
    }
}
