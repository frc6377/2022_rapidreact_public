package frc.robot.common.config.robots;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.common.config.Config;

public class Omnibot extends Config {
  // TODO: Put the correct ids here
  {
    robotName = "OmniBot";

    // --------------------------- Hardware on robot and types ---------------------------
    shooterConfig.simulate = false;
    turretConfig.simulated = false;
    pneumaticConfig.simulated = false;
    intakeConfig.simulated = false; // Is dependt on Pneumatics being enabled
    signalingConfig.simulated = true;

    pneumaticConfig.pcmType = PneumaticsModuleType.CTREPCM;

    // --------------------------- IDs ---------------------------
    tankdriveLayout.leftLeaderID = 5;
    tankdriveLayout.rightLeaderID = 4;
    tankdriveLayout.leftFollowerID = 3;
    tankdriveLayout.rightFollowerID = 2;

    shooterConfig.flywheelID = 9;
    shooterConfig.flywheelFollowID = 11;
    turretConfig.turretId = 30;

    pneumaticConfig.compressorID = 0;

    intakeConfig.soleniodExtensionModuleId = 1;
    intakeConfig.soleniodRetrationModuleId = 0;

    intakeConfig.rollerId = 44;
    intakeConfig.pneumaticControllerID = 0;

    // --------------------------- Random semi-important numbers ---------------------------
    shooterConfig.kf = 0.1;
    shooterConfig.kp = 0;
    shooterConfig.shooterInvertIsClockwise = true;
    shooterConfig.followMasterInvert = false;
    shooterConfig.idleVelo = 1000;

    turretConfig.ticsToDegrees = 1 / 150;
    turretConfig.turretRange = 90;
    turretConfig.degreesToTics = 150;
    turretConfig.invertIsClockwise = true;
    turretConfig.kF = 0.051291;
    turretConfig.kP = 0.354916;
    turretConfig.maxAccel = 500;
    turretConfig.maxVelo = 1000;

    intakeConfig.update();
  }
}
