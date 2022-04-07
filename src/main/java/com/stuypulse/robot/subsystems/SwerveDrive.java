// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.Constants;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import static com.stuypulse.robot.Constants.SwerveDrive.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private AHRS gyro;

    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    // private Field2d field;

    private static SwerveModule makeModule(String id ,double sx, double sy, int drive, int pivot) {
        return new SwerveModule(id, new Vector2D(sx, sy).mul(TRACK_SIZE), drive, pivot);
    }

    public SwerveDrive() {
        modules = new SwerveModule[] {
            makeModule("TR", +0.5, +0.5, Ports.TOP_RIGHT_DRIVE, Ports.TOP_RIGHT_PIVOT),
            makeModule("TL", -0.5, +0.5, Ports.TOP_LEFT_DRIVE, Ports.TOP_LEFT_PIVOT),
            makeModule("BL", -0.5, -0.5, Ports.BOTTOM_LEFT_DRIVE, Ports.BOTTOM_LEFT_PIVOT),
            makeModule("BR", +0.5, -0.5, Ports.BOTTOM_RIGHT_DRIVE, Ports.BOTTOM_RIGHT_PIVOT),
        };

        kinematics = new SwerveDriveKinematics(
            // TODO: replace with a stream
            modules[0].getLocation().getTranslation2d(),
            modules[1].getLocation().getTranslation2d(),
            modules[2].getLocation().getTranslation2d(),
            modules[3].getLocation().getTranslation2d()
        );

        odometry = new SwerveDriveOdometry(kinematics, Angle.kZero.getRotation2d());
        
        gyro = new AHRS(SPI.Port.kMXP);
    }

    /******************
     * MODULE CONTROL *
     ******************/

    public void drive(Vector2D velocity, double angular) {
        velocity = velocity.rotate(getAngle().negative());

        double maxMag = 0.0;
        for (SwerveModule module : modules) {
            double mag = module.setTarget(velocity, angular);
            if (mag > maxMag) maxMag = mag;
        }

        if (maxMag > Constants.SwerveModule.MAX_VELOCITY) {
            for (SwerveModule module : modules) {
                module.normalizeTarget(maxMag, Constants.SwerveModule.MAX_VELOCITY);
            }
        }
    }

    public void setStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; ++i) {
            modules[i].setTarget(states[i]);
        }
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void reset() {
        for (SwerveModule module : modules) {
            module.reset();
        }
    }

    /****************
     * ANGLE & GYRO *
     ****************/

    public double getRawAngle() {
        return gyro.getAngle();
    }

    public Angle getAngle() {
        return Angle.fromDegrees(getRawAngle());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getRawAngle());
    }

    public void resetGyro() {
        gyro.reset();
    }

    /************
     * ODOMETRY *
     ************/

    public Pose2d getPose() {
        updatePose();
        return odometry.getPoseMeters();
    }

    private void updatePose() {
        odometry.update(
            getRotation2d(), 
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState()
        );
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }


    public SwerveModule getModule(String id) {
        for (SwerveModule module : modules) {
            if(id.equals(module.getID())) return module;
        }

        return null;
    }

    public SwerveModule getModule(int index) {
        if (index>=0 && index<modules.length) {
            return modules[index];
        }
        return null;
    }

    @Override
    public void periodic() {
        updatePose();
    }
}
