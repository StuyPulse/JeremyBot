package com.stuypulse.robot.constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface Motion {
    // double MAX_VELOCITY = Modules.MAX_SPEED;
    // double MAX_ACCELERATION = 2.5;

    public interface Odometry {
        Pose2d START = new Pose2d(0, 0, new Rotation2d());
    
        Matrix<N3, N1> STATE_STD = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0);
        Matrix<N1, N1> MEASUREMENT_STD = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0);
        Matrix<N3, N1> VISION_STD = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0);
    }

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