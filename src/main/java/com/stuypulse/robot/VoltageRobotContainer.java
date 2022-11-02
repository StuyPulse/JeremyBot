/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot;

import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.commands.autos.*;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.VoltageSwerveModule;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.input.gamepads.PS4;
import com.stuypulse.stuylib.input.gamepads.keyboard.SimKeyGamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class VoltageRobotContainer {

    private interface Chassis {
        double WIDTH = Units.inchesToMeters(29.0);
        double HEIGHT = Units.inchesToMeters(29.0);
        double MAX_SPEED = Units.feetToMeters(14.0);
    }

    private interface FrontRight {
        String ID = "Front Right";
        int DRIVE_PORT = 3;
        int TURN_PORT = 4;
        int ENCODER_PORT = 1;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(143));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
    }

    private interface FrontLeft {
        String ID = "Front Left";
        int DRIVE_PORT = 1;
        int TURN_PORT = 2;
        int ENCODER_PORT = 3;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(36));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
    }

    private interface BackLeft {
        String ID = "Back Left";
        int DRIVE_PORT = 5;
        int TURN_PORT = 6;
        int ENCODER_PORT = 2;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(-80.5));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
    }

    private interface BackRight {
        String ID = "Back Right";
        int DRIVE_PORT = 7;
        int TURN_PORT = 8;
        int ENCODER_PORT = 0;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(142.3));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
    }

    private static VoltageSwerveModule makeModule(String id, int turnId, int driveId, int encoderPort,
            SmartAngle absoluteOffset, Translation2d moduleOffset) {
        return new VoltageSwerveModule(id, moduleOffset, turnId, encoderPort, absoluteOffset, driveId);
    }

    // Subsystems
    public final VoltageSwerveModule[] modules = new VoltageSwerveModule[] {
            makeModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT,
                    FrontRight.ENCODER_PORT, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
            makeModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT,
                    FrontLeft.ENCODER_PORT, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
            makeModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT,
                    BackLeft.ENCODER_PORT, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
            makeModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT,
                    BackRight.ENCODER_PORT, BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET)
    };

    // Gamepads
    // public final Gamepad driver = new SimKeyGamepad(); // new
    // BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad driver = new PS4(0);
    public final Gamepad test = new AutoGamepad(Ports.Gamepad.TEST);

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    public VoltageRobotContainer() {
        // Disabling joystick connect allows for AutoGamepad to not be noisy
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure the button bindings
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    private void configureDefaultCommands() {

        // swerve.setDefaultCommand(new InstantCommand(() -> swerve.setStates(new
        // Vector2D(0, 0.5), 0)));
    }

    private void configureButtonBindings() {
    }

    public void configureAutons() {
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