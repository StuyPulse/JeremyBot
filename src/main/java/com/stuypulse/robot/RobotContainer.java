/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.*;

import java.util.List;

import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.autos.*;
import com.stuypulse.robot.constants.Controls;
import com.stuypulse.robot.constants.Modules;
import com.stuypulse.robot.constants.Modules.*;
import com.stuypulse.robot.subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
    public final Gamepad driver = new AutoGamepad(Controls.Ports.DRIVER);
    
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
        // swerve.setDefaultCommand(new DriveCommand(swerve, driver));
        swerve.setDefaultCommand(new TurnModule(swerve, driver));
    }

    private void configureButtonBindings() {
        driver.getTopButton().whileHeld(new ResetModule(swerve, swerve.getModule(TopRight.ID), driver));
        driver.getLeftButton().whileHeld(new ResetModule(swerve, swerve.getModule(TopLeft.ID), driver));
        driver.getBottomButton().whileHeld(new ResetModule(swerve, swerve.getModule(BottomLeft.ID), driver));
        driver.getRightButton().whileHeld(new ResetModule(swerve, swerve.getModule(BottomRight.ID), driver));
    
        driver.getDPadUp().whileHeld(new ControlModule(swerve, swerve.getModule(TopRight.ID), driver));
        driver.getDPadLeft().whileHeld(new ControlModule(swerve, swerve.getModule(TopLeft.ID), driver));
        driver.getDPadDown().whileHeld(new ControlModule(swerve, swerve.getModule(BottomLeft.ID), driver));
        driver.getDPadRight().whileHeld(new ControlModule(swerve, swerve.getModule(BottomRight.ID), driver));
    }

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuto());
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return autonChooser.getSelected();
    
        return new FollowTrajectory(
            swerve, 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, new Rotation2d(0)), List.of(), new Pose2d(5, -0.5, new Rotation2d(0.0)),
                new TrajectoryConfig(Modules.MAX_SPEED, Modules.MAX_ACCEL).addConstraint(
                    new SwerveDriveKinematicsConstraint(swerve.getKinematics(), Modules.MAX_SPEED)
                )
            )
        );
    }

}