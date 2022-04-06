// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.Vector2D;

import static com.stuypulse.robot.Constants.SwerveDrive.*;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private AHRS gyro;

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
        normalizeModulePositions();

        gyro = new AHRS(SPI.Port.kMXP);
    }

    private void normalizeModulePositions() {
        double maxDist = 0;
        for (SwerveModule module : modules) {
            double dist = module.getLocation().distance();
            if (dist > maxDist) maxDist = dist;
        }

        for (SwerveModule module : modules) {
            module.normalizeLocation(maxDist);
        }
    }

    public void SetModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
            SwerveModuleState state = moduleStates[i];
            modules[i].setSpeed(state.speedMetersPerSecond);
            modules[i].setTarget(new Polar2D(Integer.MAX_VALUE, Angle.fromDegrees(state.angle.getDegrees())));
        }
    }

    public void drive(Vector2D translation, double angular) {
        translation = translation.rotate(getAngle().negative());

        double maxMag = 1.0;
        for (SwerveModule module : modules) {
            double mag = module.setTarget(translation, angular);
            if (mag > maxMag) maxMag = mag;
        }

        for (SwerveModule module : modules) {
            module.normalizeTarget(maxMag);
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void reset() {
        for (SwerveModule module : modules) {
            module.reset();
        }
    }

    public double getRawAngle() {
        return gyro.getAngle();
    }

    public Angle getAngle() {
        return Angle.fromDegrees(getRawAngle());
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
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
}
