package com.stuypulse.robot.subsystems.modules.implementations.feedforwardback;

import com.stuypulse.robot.subsystems.modules.DriveControl;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FFBDriveControl extends DriveControl {
    private final SimpleMotorFeedforward feedforward;
    private final Controller feedback;
    
    private double targetVelocity;
    private double previousTarget;

    private StopWatch timer;
    
    public FFBDriveControl(PhysicalControl physicalControl, SimpleMotorFeedforward feedforward, Controller feedback) {
        super(physicalControl);

        this.feedforward = feedforward;
        this.feedback = feedback;
        
        this.targetVelocity = 0.0;
        this.previousTarget = 0.0;
        
        this.timer = new StopWatch();
    }

    @Override
    public void setVelocity(double velocity) {
        previousTarget = targetVelocity;
        targetVelocity = velocity;
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
