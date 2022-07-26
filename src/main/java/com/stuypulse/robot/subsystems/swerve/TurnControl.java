package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnControl extends SubsystemBase {
    private Rotation2d target;
    private AngleController controller;

    public TurnControl(AngleController controller) {
        this.controller = controller;

        this.target = new Rotation2d(0.0);
    }

    public void setAngle(Rotation2d target) {
        this.target = target;
    }

    public abstract Rotation2d getAngle();

    protected abstract void setVoltage(double voltage); 

    protected abstract void reset();

    protected void log(String who) {
        SmartDashboard.putNumber(who + "/Target Angle", target.getDegrees());
        SmartDashboard.putNumber(who + "/Angle", Angle.fromRotation2d(getAngle()).toDegrees());
        SmartDashboard.putNumber(who + "/Angle Error", target.minus(getAngle()).getDegrees());
    }

    @Override
    public void periodic() {
        double outputVolts = controller.update(Angle.fromRotation2d(target), Angle.fromRotation2d(getAngle()));
        setVoltage(outputVolts);
    }

}
