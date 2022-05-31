package com.stuypulse.robot.constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public interface Odometry {
        Matrix<N3, N1> STATE_STDDEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            3.0,
            3.0,
            3.0
        );
        Matrix<N1, N1> MEASURE_STDDEV = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
            0.01
        );
        Matrix<N3, N1> VISION_STDDEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            1.0,
            1.0,
            1.0
        );
    }
}