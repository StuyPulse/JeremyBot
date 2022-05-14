package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * Contains constants for a simulated swerve module
 */
public interface SimModule {
	public interface Turn {
		public interface Encoder {
			// TODO: input real values
			double GEAR_RATIO = 1;
			double DISTANCE_PER_PULSE = 1;

			int PORT_A = 0;
			int PORT_B = 1;
		}

		public interface SysID {
			double kA = 0.5;
			double kV = 0.1;

			public static LinearSystem<N1, N1, N1> getPlant() {
				return LinearSystemId.identifyVelocitySystem(kV, kA);
			}
		}

		public interface Feedback {
            double kP = 0.01;
            double kI = 0.0;
            double kD = 0.005;

            public static Controller getController() {
                return new PIDController(kP, kI, kD);
            }
		}
	}
}
