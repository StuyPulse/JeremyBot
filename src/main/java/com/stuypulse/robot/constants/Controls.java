package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.math.util.Units;

public interface Controls {
    public interface Ports {
        int DRIVER = 0;
    }

    Number MAX_TELEOP_SPEED = new SmartNumber("Controls/Max Speed (ft/s)", 14).filtered(Units::feetToMeters).number();
    Number MAX_TELEOP_ANGULAR = new SmartNumber("Controls/Max Angular (rad/s)", Modules.MAX_ANGULAR_SPEED);

    public interface Drive {
        SmartNumber DEADBAND = new SmartNumber("Controls/Drive/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Controls/Drive/RC", 0.2);
        
        public static VFilter getFilter() {
            return new VDeadZone(DEADBAND)
                .then(new VLowPassFilter(RC))
                .then(x -> x.mul(MAX_TELEOP_SPEED.doubleValue()));
        }
    }

    public interface Turn {
        SmartNumber DEADBAND = new SmartNumber("Controls/Turn/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Controls/Turn/RC", 0.2);

        public static IFilter getFilter() {
            return IFilter.create(x -> SLMath.deadband(x, DEADBAND.get()))
                .then(new LowPassFilter(RC))
                .then(x -> x * MAX_TELEOP_ANGULAR.doubleValue());
        }
    }
}
