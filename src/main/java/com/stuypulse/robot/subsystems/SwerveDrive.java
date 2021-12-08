// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;

    public SwerveDrive() {
        this.modules = new SwerveModule[] {
            // new SwerveModule(new Vector2D()),
            // new SwerveModule(new Vector2D()),
            // new SwerveModule(new Vector2d()),
            // new SwerveModule(new Vector2d()),
        };

        // TODO: add child subsystems?
    }

    public void drive(Vector2D translation, double angular) {
        double maxMag = 1.0;
        for (SwerveModule module : modules) {
            double mag = module.setTarget(translation, angular);
            if (mag > maxMag) maxMag = mag;
        }

        for (SwerveModule module : modules) {
            module.normalize(maxMag);
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
