package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.IFilterGroup;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

public interface Controls {
    public interface Ports {
        int DRIVER = 0;
    }

    double MAX_TELEOP_SPEED = Modules.MAX_SPEED / 4.0;
    double MAX_TELEOP_ANGULAR = Modules.MAX_ANGULAR_SPEED / 4.0;

    public interface Drive {
        SmartNumber DEADBAND = new SmartNumber("Controls/Drive/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Controls/Drive/RC", 0.2);
    
        public static IFilter getFilter() {
            return new IFilterGroup(
                x -> SLMath.deadband(x, DEADBAND.get()),
                new LowPassFilter(RC)
            );
        }
    }

    public interface Turn {
        SmartNumber DEADBAND = new SmartNumber("Controls/Turn/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Controls/Turn/RC", 0.2);

        public static IFilter getFilter() {
            return new IFilterGroup(
                x -> SLMath.deadband(x, DEADBAND.get()),
                new LowPassFilter(RC)
            );
        }
    }
}
