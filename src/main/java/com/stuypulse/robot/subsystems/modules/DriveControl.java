package com.stuypulse.robot.subsystems.modules;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveControl extends SubsystemBase {

    public interface PhysicalControl {
        double getVelocity();
        void setVoltage(double voltage);
        void reset();
    }

    private PhysicalControl physicalControl;

    public DriveControl(PhysicalControl physicalControl) {
        this.physicalControl = physicalControl;
    }
    
    public abstract void setVelocity(double velocity);

    @Override
    public abstract void periodic();

    protected PhysicalControl getPhysicalControl() {
        return physicalControl;
    }

    public double getVelocity() {
        return getPhysicalControl().getVelocity();
    }

    protected void setVoltage(double voltage) {
        getPhysicalControl().setVoltage(voltage);
    }

    protected void reset() {
        getPhysicalControl().reset();
    }

}
