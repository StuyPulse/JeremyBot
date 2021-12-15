// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.Vector2D;

import static com.stuypulse.robot.Constants.SwerveDrive.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;

    private static SwerveModule makeModule(double sx, double sy, int drive, int pivot) {
        return new SwerveModule(
            new Vector2D(sx, sy),
            drive,
            pivot
        );
    }

    public SwerveDrive() {
        modules = new SwerveModule[] {
            makeModule(+0.5, +0.5, Ports.TOP_RIGHT_DRIVE, Ports.TOP_RIGHT_PIVOT),
            makeModule(-0.5, +0.5, Ports.TOP_LEFT_DRIVE, Ports.TOP_LEFT_PIVOT),
            makeModule(-0.5, -0.5, Ports.BOTTOM_LEFT_DRIVE, Ports.BOTTOM_LEFT_PIVOT),
            makeModule(+0.5, -0.5, Ports.BOTTOM_RIGHT_DRIVE, Ports.BOTTOM_RIGHT_PIVOT),
        };
        normalizeModulePositions();

        // TODO: add child subsystems?
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

    public void drive(Vector2D translation, double angular) {
        double maxMag = 1.0;
        for (SwerveModule module : modules) {
            double mag = module.setTarget(translation, angular);
            if (mag > maxMag) maxMag = mag;
        }

        for (SwerveModule module : modules) {
            module.normalizeTarget(maxMag);
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
