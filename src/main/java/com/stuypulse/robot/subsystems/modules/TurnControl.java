package com.stuypulse.robot.subsystems.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnControl extends SubsystemBase {

    public interface PhysicalControl {
        Rotation2d getAngle();
        void setVoltage(double voltage);
        void reset();
    }

    private PhysicalControl physicalControl;

    public TurnControl(PhysicalControl physicalControl) {
        this.physicalControl = physicalControl;
    }

    public abstract void setAngle(Rotation2d target);

    @Override
    public abstract void periodic();

    protected PhysicalControl getPhysicalControl() {
        return physicalControl;
    }

    public Rotation2d getAngle() {
        return getPhysicalControl().getAngle();
    }

    protected void setVoltage(double voltage) {
        getPhysicalControl().setVoltage(voltage);
    }

    protected void reset() {
        getPhysicalControl().reset();
    }

}
