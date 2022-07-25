package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveControl extends SubsystemBase {
    private final SimpleMotorFeedforward feedforward;
    private final PIDController feedback;
    
    private double targetVelocity;
    private double previousTarget;

    private StopWatch timer;

    public DriveControl(SimpleMotorFeedforward feedforward, PIDController feedback) {
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

    public abstract double getVelocity();

    protected abstract void setVoltage(double voltage);
    
    protected abstract void reset();

    protected void log(String who) {
        SmartDashboard.putNumber(who + "/Target Velocity", targetVelocity);
        SmartDashboard.putNumber(who + "/Velocity", getVelocity());
        SmartDashboard.putNumber(who + "/Velocity Error", targetVelocity - getVelocity());
    }

    @Override
    public void periodic() {
        double outputVolts = 
            feedforward.calculate(previousTarget, targetVelocity, timer.reset()) + 
            feedback.calculate(targetVelocity, getVelocity());

        setVoltage(outputVolts);

        previousTarget = targetVelocity;
    }

}
