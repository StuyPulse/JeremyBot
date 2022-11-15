package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.filters.IFilter;

public class DelayFilter implements IFilter {

    private double prevAngle;

    public DelayFilter() {
        prevAngle = 0.0;
    }

    @Override
    public double get(double next) {
        var shitfart = prevAngle;
        prevAngle = next;
        return shitfart;
    }
    
}
