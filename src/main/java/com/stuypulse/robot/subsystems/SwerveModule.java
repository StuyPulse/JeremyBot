package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public String getId();
    public Translation2d getLocation();
    public void setTargetState(SwerveModuleState state);
    public SwerveModuleState getState();
}
