package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.control.Controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnControl extends SubsystemBase {
    private Rotation2d target;
    private Controller feedback;

    public TurnControl(Controller feedback) {
        this.feedback = feedback;

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
        SmartDashboard.putNumber(who + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(who + "/Angle Error", target.minus(getAngle()).getDegrees());
    }

    @Override
    public void periodic() {
        double outputVolts = 
            feedback.update(target.minus(getAngle()).getDegrees());

        setVoltage(outputVolts);
    }

}
