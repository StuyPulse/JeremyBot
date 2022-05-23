package com.stuypulse.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface Motion {

    double dt = 0.020;
    // double MAX_VELOCITY = Modules.MAX_SPEED;
    // double MAX_ACCELERATION = 2.5;

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

        public static ProfiledPIDController getController() {
            return new ProfiledPIDController(
                kP, kI, kD, 
                new Constraints(Modules.MAX_ANGULAR_SPEED, Modules.MAX_ANGULAR_ACCEL)
            );
        }
    }
}