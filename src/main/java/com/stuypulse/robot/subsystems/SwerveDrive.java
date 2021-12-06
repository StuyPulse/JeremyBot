// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private SwerveModule[] modules;

  public SwerveDrive() {
    this.modules = {
      new SwerveModule(new Vector2d()),
      new SwerveModule(new Vector2d()),
      new SwerveModule(new Vector2d()),
      new SwerveModule(new Vector2d()),
    }
  }

  public void drive(Vector2D translation, double angular) {
    double maxMagnitude = 1.0;
    for (SwerveModule module : modules) {
      Polar2D poop = module.getTargetValue(translation, angular);
      maxMagnitude = Math.max(poop.magnitude, maxMagnitude);
      // maxMagnitude = Math.max(maxMagnitude, module.setTarget(translation, angular));
      // calculate target vector
    }

    for (SwerveModule module : modules) {
      // post-normalization
    }
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
