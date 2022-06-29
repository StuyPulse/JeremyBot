package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.SLMath;

/** Contains contants about the SRX Mag Encoder on our swerve modules */
public interface MagEncoder {
    double MIN_INPUT = 0.0;
    double MAX_INPUT = 1.0;
    
    double MIN_OUTPUT = -Math.PI;
    double MAX_OUTPUT = +Math.PI;

    public static double getRadians(double input) {
        return SLMath.lerp(MIN_INPUT, MAX_INPUT, (input - MIN_INPUT) / (MAX_INPUT - MIN_INPUT));
    }
}

