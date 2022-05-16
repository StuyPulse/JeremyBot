package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveControl extends SubsystemBase {
    public abstract void setVelocity(double velocity);

    public abstract double getVelocity();

    protected abstract void setVoltage(double voltage);
    
    protected abstract void reset();
}
