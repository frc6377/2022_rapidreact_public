package frc.robot.systems.turret;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.common.motorfactory;
import frc.robot.systems.turret.HoodSubsystem.HoodState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.utilities.GunnerRumble;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterConfig config;
    private final LimelightSubsystem limelight;
    private ShooterState shooterState = ShooterState.IDLE;
    private final ShooterTab shooterTab = new ShooterTab();
    private final HoodSubsystem hood;
    private final WPI_TalonFX flywheel;

    private final ShooterHardware hardware;

    public ShooterSubsystem(
            ShooterConfig config,
            LimelightSubsystem limelight,
            HoodSubsystem hood) {
        this.config = config;
        this.limelight = limelight;
        this.hood = hood;
        this.hardware = new ShooterHardware(config);
        this.flywheel = this.hardware.flywheel;

        addChild("flywheel Lead", flywheel);
        addChild("flywheel Follow", this.hardware.flywheelFollow);
    }

    public enum ShooterState {
        TARGETING,
        IDLE
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public void periodic() {
        var rumble = setRumble();
        GunnerRumble.setRumble(rumble);
        hood.setState(HoodState.CLOSE); // finds new hood state based on limelight distance
        limelight.setLongRange(hood.GetState() == HoodState.LONG); // sends new hood state to limelight
        hood.setHoodAngle(hood.GetState()); // actually actuates the servos
        var velocity = shooterVelocity();
        setMotorValue(flywheel.getSelectedSensorVelocity(), velocity);

        var state = updateDashboard();
        shooterTab.updateShooterTab(velocity, state, rumble > 0.001, limelight.getDist());
    }

    private void setMotorValue(double currentVelocity, double targetVelocity) {
        if (shooterState == ShooterState.IDLE && shouldCoast(currentVelocity, targetVelocity)) {
            flywheel.set(ControlMode.PercentOutput, 0);
        } else {
            flywheel.set(ControlMode.Velocity, targetVelocity);
        }
    }

    /**
     * This function looks to see if we should continue coast down.
     * If we are above the target by more than 10%, we want to coast.
     *
     * @param currentVelocity is the current velocity that the flywheel is running.
     * @param targetVelocity  is the ideal velocity that we intend to hit.
     * @return returns true when we should coast and false when we should not.
     */
    private boolean shouldCoast(double currentVelocity, double targetVelocity) {
        var percentDifference = Math.abs((targetVelocity - currentVelocity) / targetVelocity);
        var tenPercentAboveTarget = percentDifference > 0.1;

        return tenPercentAboveTarget;
    }

    private double shooterVelocity() {
        var shooterVelo = config.idleVelo;

        if (ShooterState.TARGETING == shooterState) {
            var distance = limelight.hasValidTarget() ? limelight.getDist() : config.rangeWithoutTarget;
            shooterVelo = computeShooterVelo(distance);
        }

        return shooterVelo + SmartDashboard.getNumber("shooter velo offset", 0);
    }

    private double computeShooterVelo(double distanceToTarget) {
        return (hood.GetState() == HoodState.LONG) ? longHoodEquation(distanceToTarget) : closeHoodEquation(distanceToTarget);
    }

    private double closeHoodEquation(double distanceToTarget) {
        // equation derived from data from
        // https://docs.google.com/spreadsheets/d/18Gqk_lXow_DTrNU5hVca0G0iTqapNoELlDuWBjtFmV4/edit?usp=sharing
        // current shooter tab
        return (48.1 * distanceToTarget) + 7292;
    }

    private double longHoodEquation(double distanceToTarget) {
        // equation derived from data from
        // https://docs.google.com/spreadsheets/d/1MOAkfEtbHe1HUFGGm3kArj8dxei0DrvGtOlRAt-DD9c/edit?usp=sharing
        // pre-austin tab
        return (39.4 * distanceToTarget) + 7044;
    }

    private double setRumble() {
        if (ShooterState.TARGETING == shooterState && readyToShoot()) {
            return 0.3;
        }
        return 0.0;
    }

    private String updateDashboard() {
        SmartDashboard.putString("hood state", hood.GetState().toString());
        String temp = "error";
        if (shooterState == ShooterState.IDLE) {
            temp = "idle";
        } else if (shooterState == ShooterState.TARGETING) {
            temp = "targeting";
        }
        SmartDashboard.putString("shooter state", temp);
        return temp;
    }

    private boolean readyToShoot() {
        var error = flywheel.getClosedLoopError();
        return (Math.abs(error) <= config.rumbleVelocityTolerence
                && limelight.hasValidTarget()
                && Math.abs(limelight.getXOffset()) <= config.rumbleOffsetTolerance);
    }

    private class ShooterTab {
        public static final String ShooterTabName = "Shooter";

        private final NetworkTableEntry velocity;
        private final NetworkTableEntry state;
        private final NetworkTableEntry distance;
        private final NetworkTableEntry rumble;

        private static final String distance_desc = "Distance (in): ";
        private static final String state_desc = "State: ";
        private static final String velocity_desc = "Velocity: ";
        private static final String rumble_desc = "Rumbler: ";

        public ShooterTab() {
            var layout = Shuffleboard.getTab(ShooterTabName)
                    .getLayout("Shooter Info", BuiltInLayouts.kList)
                    .withSize(2, 2)
                    .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

            distance = layout.add("shooter_distance", distance_desc).getEntry();
            rumble = layout.add("shooter_rumble", rumble_desc).getEntry();
            velocity = layout.add("shooter_velocity", velocity_desc).getEntry();
            state = layout.add("shooter_state", state_desc).getEntry();
        }

        public void updateShooterTab(
                double velocity,
                String state,
                boolean rumble,
                double distance) {
            this.velocity.setString(velocity_desc + velocity);
            this.state.setString(state_desc + state);
            this.rumble.setString(rumble_desc + rumble);
            this.distance.setString(distance_desc + distance);
        }
    }

    private class ShooterHardware {
        public WPI_TalonFX flywheel;
        private WPI_TalonFX flywheelFollow;

        public ShooterHardware(ShooterConfig cfg) {
            if (cfg.simulate) {
                return;
            }
            var flywheelConfig = new motorfactory.MotorConfiguration();
            var flywheelFollowConfig = new motorfactory.MotorConfiguration();

            flywheelConfig.talonFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 1, 1, 1);
            flywheelConfig.talonFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 60, 60, 1);
            flywheelFollowConfig.talonFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 1, 1, 1);
            flywheelFollowConfig.talonFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 60, 60, 1);

            invert(cfg, flywheelConfig, flywheelFollowConfig);
            setClosedLoopGains(cfg, flywheelConfig);

            flywheelConfig.neutralMode = NeutralMode.Coast;
            flywheelFollowConfig.neutralMode = NeutralMode.Coast;

            flywheelConfig.talonFXConfig.primaryPID.selectedFeedbackSensor =
                    TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

            flywheel = motorfactory.createTalonFX(cfg.flywheelID, flywheelConfig);
            flywheelFollow = motorfactory.createTalonFX(cfg.flywheelFollowID, flywheelFollowConfig);

            flywheelFollow.follow(flywheel);
        }

        private void invert(
                ShooterConfig cfg,
                motorfactory.MotorConfiguration motorConfig,
                motorfactory.MotorConfiguration followConfig) {
            motorConfig.invertType = TalonFXInvertType.CounterClockwise;
            if (cfg.shooterInvertIsClockwise) motorConfig.invertType = TalonFXInvertType.Clockwise;

            followConfig.invertType = TalonFXInvertType.OpposeMaster;
            if (cfg.followMasterInvert) followConfig.invertType = TalonFXInvertType.FollowMaster;
        }

        private void setClosedLoopGains(ShooterConfig cfg, motorfactory.MotorConfiguration motorConfig) {
            motorConfig.talonFXConfig.slot0.kF = cfg.kf;
            motorConfig.talonFXConfig.slot0.kP = cfg.kp;
            motorConfig.talonFXConfig.slot0.kD = cfg.kD;
        }
    }

}
