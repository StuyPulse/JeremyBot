package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * for the purpose of swapping out various types of modules
 * when simulating -- not a good idea once a module implementation
 * that works on a robot is found
 */
public interface SwerveModule {
    public String getId();
    public Translation2d getModuleOffset();
    public void setTargetState(SwerveModuleState state);
    public SwerveModuleState getState();
}
