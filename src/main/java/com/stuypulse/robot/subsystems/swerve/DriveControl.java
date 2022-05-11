package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveControl extends SubsystemBase {
    private final SimpleMotorFeedforward feedforward;
    private final Controller feedback;
    
    private double targetVelocity;
    private double previousTarget;

    private StopWatch timer;
    
    public DriveControl(SimpleMotorFeedforward feedforward, Controller feedback) {
        this.feedforward = feedforward;
        this.feedback = feedback;
        
        this.targetVelocity = 0.0;
        this.previousTarget = 0.0;
        
        this.timer = new StopWatch();
    }

    public void setVelocity(double velocity) {
        previousTarget = targetVelocity;
        targetVelocity = velocity;
    }

    public double getVelocity() {
        return 0.0;
    }

    protected void setVoltage(double voltage) {
        
    }

    @Override
    public void periodic() {
        double outputVolts = 
            feedforward.calculate(previousTarget, targetVelocity, timer.reset()) + 
            feedback.update(targetVelocity, getVelocity());

        setVoltage(outputVolts);

        previousTarget = targetVelocity;
    }

}
