// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.systems.drivetrain.commands.autonomous.AutonomousFactory;
import frc.robot.systems.drivetrain.commands.ChangeGear;
import frc.robot.systems.climber.commands.autoFunctions.StopAutoClimb;
import frc.robot.systems.turret.commands.StopTargeting;
import frc.robot.systems.turret.commands.Target;
import frc.robot.common.dashboard.ShuffleboardHelper;
import frc.robot.systems.drivetrain.steering.TrajectoryBuilder;
import frc.robot.systems.signaling.SignalingSubsystem;
import frc.robot.common.utilities.GunnerRumble;
import frc.robot.common.utilities.TriggerSupplier;
import frc.robot.common.utilities.leds.patterns.AutoToManual;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final AutonomousFactory autonomousFactory;
    private final RobotSystems systems;
    private SendableChooser<AutonomousFactory.CommandName> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotSystems systems) {
        Logger.configureLoggingAndConfig(this, false);
        SignalingSubsystem.getInstance();

        this.systems = systems;

        ShuffleboardHelper shuffleboardHelper = new ShuffleboardHelper();
        shuffleboardHelper.register();

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("5 ball auto", AutonomousFactory.CommandName.FiveBallAuto);
        autoChooser.addOption("2 ball auto truss", AutonomousFactory.CommandName.AutoBlueBallCommandGroup);
        autoChooser.addOption("2 ball auto terminal", AutonomousFactory.CommandName.AutoBlueBallCommandGroup2);
        autoChooser.addOption("1 ball auto", AutonomousFactory.CommandName.OneBallAuto);
        SmartDashboard.putData(autoChooser);

        SmartDashboard.putNumber("shooter velo offset", 0);

        // Start the compressor
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> systems.Compressor.start()));

        // Create the factory of commands that we will be using for autonomous
        this.autonomousFactory = new AutonomousFactory(systems, systems.getConfig(), new TrajectoryBuilder(systems.getConfig().tankdriveLayout));

        // Set up rumble system
        GunnerRumble.setController(systems.GunnerController);

        // Configure the button bindings
        configureButtonBindings(systems.DriverController, systems.GunnerController);

        // Configure default commands
        // Set the default drive command to split-stick arcade drive

        if (!systems.getConfig().tankdriveLayout.simulated)
            systems.DriveTrain.DriveTrainSubsystem.setDefaultCommand(
                    systems.DriveTrain.DefaultCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(XboxController driver, XboxController gunner) {
        TriggerSupplier.getLeftBumper(driver).whenActive(systems.getCommands().toggleIntake());

        Trigger leftTriggerDriver = TriggerSupplier.getLeftTrigger(driver, systems.getConfig().Driver.triggerSensitivity)
                .whenActive(systems.getCommands().intakeCargo())
                .whileActiveOnce(systems.getCommands().ballHandlerForward())
                .whenInactive(systems.getCommands().ballHandlerNeutral());
        Trigger rightTriggerDriver = TriggerSupplier.getRightTrigger(driver, systems.getConfig().Driver.triggerSensitivity)
                .whenActive(systems.getCommands().ejectCargo())
                .whileActiveOnce(systems.getCommands().ballHandlerReverse())
                .whenInactive(systems.getCommands().ballHandlerNeutral());
        TriggerSupplier.getRightBumper(driver).whileActiveOnce(new ChangeGear(false)).whenInactive(new ChangeGear(true));

        TriggerSupplier.upDPAd(driver).whenActive(
                () -> SignalingSubsystem.getInstance().schedulePattern(new AutoToManual(System.currentTimeMillis(), 2)));

        // When not moving either direction, neutral rollers
        new Trigger(leftTriggerDriver).or(rightTriggerDriver).negate().whileActiveOnce(systems.getCommands().idleRollers());

        TriggerSupplier.getYButton(gunner).whileActiveOnce(systems.Climber.partialUp);
        TriggerSupplier.getAButton(gunner).whileActiveOnce(systems.Climber.partialDown);

        TriggerSupplier.getBButton(gunner).whileActiveOnce(systems.Climber.autoNext);
        TriggerSupplier.getXButton(gunner).whileActiveOnce(systems.Climber.autoPrev);

        TriggerSupplier.getRightBumper(gunner).whenActive(systems.Climber.zeroClimber);

        TriggerSupplier.getRightTrigger(gunner, systems.getConfig().Driver.triggerSensitivity)
                .whileActiveOnce(systems.getCommands().upgoerUp())
                .whileActiveOnce(systems.getCommands().ballHandlerForward())
                .whenInactive(systems.getCommands().upgoerNeutral())
                .whenInactive(systems.getCommands().ballHandlerNeutral());

        TriggerSupplier.getLeftBumper(gunner)
                .whileActiveOnce(systems.getCommands().ballHandlerReverse())
                .whenInactive(systems.getCommands().ballHandlerNeutral());

        TriggerSupplier.getBackButton(gunner).whenActive(new StopAutoClimb(systems.Climber));
        TriggerSupplier.getStartsButton(gunner).whenActive(systems.Climber.autoClimb);
        TriggerSupplier.getLeftTrigger(gunner, systems.getConfig().Driver.triggerSensitivity)
                .whileActiveOnce(new Target(systems))
                .whenInactive(new StopTargeting(systems));

        TriggerSupplier.getLeftBumper(gunner).whenActive(systems.Climber.setUpForClimb);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousFactory.get(autoChooser.getSelected());
    }

    public void haltActuation() {
        Command haltActuation = new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new StopTargeting(systems),
                systems.getCommands().ballHandlerNeutral(),
                systems.getCommands().idleRollers(),
                systems.getCommands().upgoerNeutral());

        haltActuation.schedule();
    }

    public void setRobotNorth(double robotNorth) {
        if (systems.getConfig().turretConfig.simulated)
            return;
        systems.Turret.setRobotNorth(robotNorth);
    }

    public void setAutoOffset(double autoOffset) {
        if (systems.getConfig().turretConfig.simulated)
            return;
        systems.Turret.setAutoOffset(autoOffset);
    }
}
