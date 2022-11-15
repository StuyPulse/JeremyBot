/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.modules.VoltageSwerveModule;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class VoltageRobot extends TimedRobot {

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
    private final VoltageSwerveModule[] modules = new VoltageSwerveModule[] {
            makeModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT,
                    FrontRight.ENCODER_PORT, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
            makeModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT,
                    FrontLeft.ENCODER_PORT, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
            makeModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT,
                    BackLeft.ENCODER_PORT, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
            makeModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT,
                    BackRight.ENCODER_PORT, BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET)
    };

    private SmartNumber voltage;

    public VoltageRobot() {
        super(Settings.dT);
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        voltage = new SmartNumber("Voltage", 0.0);
        DataLogManager.start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        for (var module : modules) {
            module.setVoltage(voltage.get());
        }
    }

    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}