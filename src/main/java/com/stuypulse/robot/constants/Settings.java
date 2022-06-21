/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.IFilterGroup;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import com.stuypulse.robot.util.SmartPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Path;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

    SmartBoolean DEBUG_MODE = new SmartBoolean("Debug Mode", true);

    SmartBoolean ENABLE_WARNINGS = new SmartBoolean("Enable Warnings", true);

    static void reportWarning(String warning) {
        if (ENABLE_WARNINGS.get()) {
            DriverStation.reportWarning(warning, false);
        }
    }

    public interface Limelight {
        int[] PORTS = { 5800, 5801, 5802, 5803, 5804, 5805 };

        // characteristics of the limelight itself
        double LIMELIGHT_HEIGHT = Units.inchesToMeters(41.506);
        SmartNumber LIMELIGHT_PITCH = new SmartNumber("Limelight/Pitch", 27.0);
        SmartNumber LIMELIGHT_YAW = new SmartNumber("Limelight/Yaw", 0.0);

        // additional offsets
        SmartNumber RING_YAW = new SmartNumber("Limelight/Ring Yaw", 6.50694);
        SmartNumber PAD_YAW = new SmartNumber("Limelight/Pad Yaw", 5.0);

        // if the intake is on the ring, distance of limelight to hub
        double CENTER_TO_HUB = Field.Hub.UPPER_RADIUS;
        double LIMELIGHT_TO_INTAKE = Units.inchesToMeters(30);
        IStream RING_DISTANCE = new SmartNumber("Limelight/Ring Distance", 150).filtered(Units::inchesToMeters);
        IStream PAD_DISTANCE = new SmartNumber("Limelight/Pad Distance", 217).filtered(Units::inchesToMeters);
        double HEIGHT_DIFFERENCE = Field.Hub.HEIGHT - LIMELIGHT_HEIGHT;

        // Bounds for Distance
        double MIN_VALID_DISTANCE = Units.feetToMeters(2);
        double MAX_VALID_DISTANCE = Field.LENGTH / 2.0;

        // How long it takes to stop aligning
        double DEBOUNCE_TIME = 0.2;

        // What angle error should make us start distance alignment
        SmartNumber MAX_ANGLE_FOR_MOVEMENT = new SmartNumber("Limelight/Max Angle For Distance", 3.0);

        SmartNumber MAX_ANGLE_ERROR = new SmartNumber("Limelight/Max Angle Error", 2);
        SmartNumber MAX_DISTANCE_ERROR = new SmartNumber("Limelight/Max Distance Error", Units.inchesToMeters(6));
        SmartNumber MAX_VELOCITY = // THERE WAS AN ERROR WHERE THIS WOULD'NT CHECK WHEN MOVING BACKWARDS
                new SmartNumber("Limelight/Max Velocity Error", Units.inchesToMeters(3));
    }

    public interface Alignment {

        SmartNumber SPEED_ADJ_FILTER = new SmartNumber("Drivetrain/Alignment/Speed Adj RC", 0.1);
        SmartNumber FUSION_FILTER = new SmartNumber("Drivetrain/Alignment/Fusion RC", 0.3);

        public interface Speed {
            double kP = 2.7;
            double kI = 0;
            double kD = 0.3;

            double BANG_BANG = 0.7;

            SmartNumber ERROR_FILTER = new SmartNumber("Drivetrain/Alignment/Speed/Error Filter", 0.0);
            SmartNumber OUT_FILTER = new SmartNumber("Drivetrain/Alignment/Speed/Output Filter", 0.1);

            static Controller getController() {
                return new SmartPIDController("Drivetrain/Alignment/Speed")
                        .setControlSpeed(BANG_BANG)
                        .setPID(kP, kI, kD)
                        .setErrorFilter(new LowPassFilter(ERROR_FILTER))
                        .setOutputFilter(
                                new IFilterGroup(SLMath::clamp, new LowPassFilter(OUT_FILTER)));
            }
        }

        public interface Angle {
            double kP = 0.0366;
            double kI = 0;
            double kD = 0.0034;

            double BANG_BANG = 0.75;

            SmartNumber ERROR_FILTER = new SmartNumber("Drivetrain/Alignment/Angle/Error Filter", 0.0);
            SmartNumber OUT_FILTER = new SmartNumber("Drivetrain/Alignment/Angle/Output Filter", 0.01);

            static Controller getController() {
                return new SmartPIDController("Drivetrain/Alignment/Angle")
                        .setControlSpeed(BANG_BANG)
                        .setPID(kP, kI, kD)
                        .setErrorFilter(new LowPassFilter(ERROR_FILTER))
                        .setOutputFilter(
                                new IFilterGroup(SLMath::clamp, new LowPassFilter(OUT_FILTER)));
            }
        }
    }
}