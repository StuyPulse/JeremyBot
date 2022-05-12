package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.control.Controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurnControl extends SubsystemBase {
    private Rotation2d target;
    private Controller feedback;

    public TurnControl(Controller feedback) {
        this.feedback = feedback;

        this.target = new Rotation2d(0.0);
    }

    public void setAngle(Rotation2d target) {
        this.target = target;
    }

    public Rotation2d getAngle() {
        return new Rotation2d();
    }

    protected void setVoltage(double voltage) {

    }

    protected void reset() {
        setAngle(new Rotation2d(0.0));
    }

    @Override
    public void periodic() {
        double outputVolts = 
            feedback.update(target.minus(getAngle()).getDegrees());

        setVoltage(outputVolts);
    }

}
