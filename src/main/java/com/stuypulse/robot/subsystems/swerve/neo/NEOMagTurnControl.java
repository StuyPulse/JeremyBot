package com.stuypulse.robot.subsystems.swerve.neo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NEOMagTurnControl extends NEOTurnControl {
    private final DutyCycleEncoder absoluteEncoder;

    public NEOMagTurnControl(int turnPort, int encoderPort) {
        super(turnPort);
        absoluteEncoder = new DutyCycleEncoder(encoderPort);
    }

    // public NEOMagTurnControl configure(DutyCycleEncoderConfig config) {
    // }

    @Override
    public Rotation2d getAngle() {
        return null;
    }

    @Override
    protected void reset() {
    }
    
}
