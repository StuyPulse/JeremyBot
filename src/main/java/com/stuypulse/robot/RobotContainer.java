/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot;

import com.stuypulse.robot.commands.DriveCommand;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.commands.autos.*;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    public final Swerve swerve = new Swerve();

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER); 
                               // new AutoGamepad(Controls.Ports.DRIVER);
    
    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    public RobotContainer() {
        // Disabling joystick connect allows for AutoGamepad to not be noisy
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure the button bindings
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(swerve, driver));
    }

    private void configureButtonBindings() {
        driver.getTopButton().whenPressed(new InstantCommand(() -> swerve.reset(new Pose2d()), swerve));
    }

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuto());
        autonChooser.addOption("Test Auto", new TestAuto(this));
        autonChooser.setDefaultOption("Two Metres Auto", new TwoMetresAuto(this));
        autonChooser.addOption("Four Ball Auto", new FourBallAuto(this));
        autonChooser.addOption("Five Ball Auto", new FiveBallAuto(this));
        autonChooser.addOption("Six Ball Auto", new SixBallAuto(this));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

}