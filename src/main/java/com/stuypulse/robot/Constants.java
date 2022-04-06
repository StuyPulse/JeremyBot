// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot;

import java.nio.file.Path;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {

    public interface Ports {
        int DRIVER = 0;
    }

    public interface SwerveDrive {
        public interface Ports {
            int TOP_RIGHT_DRIVE = 1;
            int TOP_RIGHT_PIVOT = 2;

            int TOP_LEFT_DRIVE = 3;
            int TOP_LEFT_PIVOT = 4;

            int BOTTOM_LEFT_DRIVE = 5;
            int BOTTOM_LEFT_PIVOT = 6;

            int BOTTOM_RIGHT_DRIVE = 7;
            int BOTTOM_RIGHT_PIVOT = 8;
        }

        // TODO: add real track size values
        double TRACK_WIDTH = 1.0;
        double TRACK_HEIGHT = 1.0;

        Vector2D TRACK_SIZE = new Vector2D(TRACK_WIDTH, TRACK_HEIGHT);
    }

    public interface SwerveModule {

        // this converts:
        // - pivot motor revolutions into,
        // - drive wheel revolutions into,
        // - drive wheel rotation in radians
        double PIVOT_CONVERSION = 2 * Math.PI / 12.8;

        // this should convert drive ticks into meters
        double METERS_CONVERSION = 1.0;

        SmartNumber ANGLE_P = new SmartNumber("Module/AngleP", 0.1);
        SmartNumber ANGLE_I = new SmartNumber("Module/AngleI", 0.0);
        SmartNumber ANGLE_D = new SmartNumber("Module/AngleD", 0.0);

        double TARGET_DEADBAND = 0.05;
        double MIN_ALIGN_MAGNITUDE = 0.05;

        int SMART_LIMIT = 5; // amps
    }

    interface Motion {
        // top left, top right, bottom left, bottom right
        SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-0.5, 0.5),
                new Translation2d(0.5, 0.5),
                new Translation2d(-0.5, -0.5),
                new Translation2d(0.5, -0.5));

        SimpleMotorFeedforward MOTOR_FEED_FORWARD = new SimpleMotorFeedforward(FeedForward.kS, FeedForward.kV,
                FeedForward.kA);

        double MAX_VELOCITY = -1;
        double MAX_ACCELERATION = -1;

        public interface FeedForward {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
        }

        public interface X_PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }
        public interface Y_PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }
    }

    interface Odometry {
        Rotation2d STARTING_ANGLE = new Rotation2d();

        SwerveDriveOdometry ODOMETRY = new SwerveDriveOdometry(Motion.KINEMATICS, STARTING_ANGLE);
    }

    interface DriveCommand {
        SmartNumber DRIVE_RC = new SmartNumber("DriveCommand/DriveRC", 0.2);
    }

    interface Settings {
        Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
    }
}
