package com.stuypulse.robot.subsystems.modules.implementations.feedforwardback;

import com.stuypulse.robot.subsystems.modules.TurnControl;
import com.stuypulse.stuylib.control.Controller;

import edu.wpi.first.math.geometry.Rotation2d;

public class FFBTurnControl extends TurnControl {
    private Rotation2d target;
    private Controller feedback;

    public FFBTurnControl(PhysicalControl physicalControl, Controller feedback) {
        super(physicalControl);

        this.feedback = feedback;

        this.target = new Rotation2d(0.0);
    }

    @Override
    public void setAngle(Rotation2d target) {
        this.target = target;
    }

    @Override
    public void periodic() {
        double outputVolts = 
            feedback.update(target.minus(getAngle()).getDegrees());

        setVoltage(outputVolts);
    }

}
