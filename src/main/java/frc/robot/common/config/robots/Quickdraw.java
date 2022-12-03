package frc.robot.common.config.robots;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.common.config.Config;

public class Quickdraw extends Config {
  // TODO: Put the correct ids here
  {
    robotName = "QuickDraw";

    // --------------------------- Hardware on robot and types ---------------------------

    tankdriveLayout.simulated = true;
    shooterConfig.simulate = true;
    turretConfig.simulated = true;
    pneumaticConfig.simulated = true;
    intakeConfig.simulated = true; // Is dependt on Pneumatics being enabled'
    ballHandlerConfig.simulated = true;
    upgoerConfig.simulated = true;
    signalingConfig.simulated = true;
    climberConfig.simulated = true;

    pneumaticConfig.pcmType = PneumaticsModuleType.CTREPCM;

    // --------------------------- IDs ---------------------------
    tankdriveLayout.leftLeaderID = 7;
    tankdriveLayout.rightLeaderID = 8;
    tankdriveLayout.leftFollowerID = 12;
    tankdriveLayout.rightFollowerID = 13;

    tankdriveLayout.pigeonID = 3;
    tankdriveLayout.isPigeon2 = true;

    tankdriveLayout.physicalLayout.gearRatio = 7.64;
    tankdriveLayout.physicalLayout.wheelBase = 0.5842;
    tankdriveLayout.physicalLayout.wheelDiameter = 6;

    tankdriveLayout.controlParameters.ramseteBeta = 2.0;
    tankdriveLayout.controlParameters.ramseteZeta = 0.7;

    shooterConfig.flywheelID = 9;
    shooterConfig.flywheelFollowID = 11;
    turretConfig.turretId = 30;

    pneumaticConfig.compressorID = 0;

    intakeConfig.soleniodExtensionModuleId = 1;
    intakeConfig.soleniodRetrationModuleId = 0;

    intakeConfig.rollerId = 44;
    intakeConfig.pneumaticControllerID = 0;

    upgoerConfig.upgoerLeftMotorID = 9;

    // --------------------------- Random semi-important numbers ---------------------------
    shooterConfig.kf = 0.1;
    shooterConfig.kp = 0;
    shooterConfig.shooterInvertIsClockwise = true;
    shooterConfig.followMasterInvert = false;
    shooterConfig.idleVelo = 1000;

    turretConfig.ticsToDegrees = 1 / 150;
    turretConfig.turretRange = 180;
    turretConfig.degreesToTics = 150;
    turretConfig.invertIsClockwise = true;
    turretConfig.kF = 0.051291;
    turretConfig.kP = 0.354916;
    turretConfig.maxAccel = 2000;
    turretConfig.maxVelo = 4000;

    targetConfig.targetX = 8.223;
    targetConfig.targetY = 4.115;

    intakeConfig.update();
  }
}
