package com.stuypulse.robot.util;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AFilter;

public class ADelayFilter implements AFilter {

    private Angle prevAngle;

    public ADelayFilter() {
        prevAngle = Angle.kZero;
    }

    @Override
    public Angle get(Angle next) {
        Angle shitfart = prevAngle;
        prevAngle = next;
        return shitfart;
    }
    
}
