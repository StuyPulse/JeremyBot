package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveWheel extends SubsystemBase {
    
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final SimpleMotorFeedforward feedforward;
    private final PIDController feedback;

    private final StopWatch timer;
    private double targetVelocity;
    private double lastTargetVelocity;

    public DriveWheel(CANSparkMax motor, RelativeEncoder encoder, SimpleMotorFeedforward feedforward, PIDController feedback) {
        this.motor = motor;
        this.encoder = encoder;

        this.feedforward = feedforward;
        this.feedback = feedback;

        // store & calculate setpoints
        this.timer = new StopWatch();
        this.targetVelocity = 0.0;
        this.lastTargetVelocity = 0.0;
    }

    public void setVelocity(double velocity) {
        this.targetVelocity = velocity;
        this.lastTargetVelocity = velocity; 
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public void reset() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        double targetAcceleration = (targetVelocity - lastTargetVelocity) / timer.reset();

        double ff = feedforward.calculate(targetVelocity, targetAcceleration);
        double fb = feedback.update(targetVelocity - this.encoder.getVelocity());

        motor.setVoltage(SLMath.clamp(ff + fb, -16, +16));
    }

}
