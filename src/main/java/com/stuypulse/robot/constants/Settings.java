package com.stuypulse.robot.constants;

import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public interface Settings {
    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
    int UPDATE_RATE = 100;
    double dT = 1.0 / UPDATE_RATE;

    public interface Controls {
        Number MAX_TELEOP_SPEED = new SmartNumber("Controls/Max Speed (ft per s)", 14.4).filtered(Units::feetToMeters)
                .number();
        Number MAX_TELEOP_ANGULAR = new SmartNumber("Controls/Max Angular (rad per s)", 8.2);

        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Controls/Drive/Deadband", 0.05);
            SmartNumber RC = new SmartNumber("Controls/Drive/RC", 0.02);
            SmartNumber POWER = new SmartNumber("Controls/Drive/Power", 3);

            public static VFilter getFilter() {
                return new VDeadZone(DEADBAND)
                        .then(v -> new Vector2D(Math.pow(v.x, POWER.doubleValue()), Math.pow(v.y, POWER.doubleValue())))
                        .then(new VLowPassFilter(RC))
                        .then(x -> x.mul(MAX_TELEOP_SPEED.doubleValue()))
                ;
            }
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Controls/Turn/Deadband", 0.05);
            SmartNumber RC = new SmartNumber("Controls/Turn/RC", 0.02);
            SmartNumber POWER = new SmartNumber("Controls/Turn/Power", 1);

            public static IFilter getFilter() {
                return IFilter.create(x -> SLMath.deadband(x, DEADBAND.get()))
                        .then(x -> Math.pow(x, POWER.doubleValue()))
                        .then(new LowPassFilter(RC))
                        .then(x -> x * MAX_TELEOP_ANGULAR.doubleValue());
            }
        }
    }

    public interface Motion {
        double MAX_VELOCITY = 3.0;
        double MAX_ACCEL = 2.0;

        PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(MAX_VELOCITY, MAX_ACCEL);

        public interface X {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            public static PIDController getController() {
                return new PIDController(kP, kI, kD);
            }
        }

        public interface Y {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            public static PIDController getController() {
                return new PIDController(kP, kI, kD);
            }
        }

        public interface Theta {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            public static PIDController getController() {
                return new PIDController(kP, kI, kD);
            }
        }
    }
}
