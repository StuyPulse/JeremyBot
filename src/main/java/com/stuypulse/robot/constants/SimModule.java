package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * Contains constants for a simulated swerve module
 */
public interface SimModule {
	double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
	double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

	public interface Drive {
		public interface Encoder {
			double GEARING = 1.0;

			double DISTANCE_PER_PULSE = 1.0;

			int PORT_A = 10;
			int PORT_B = 11;
		}

		public interface Feedforward {
			double kS = 0;
			double kA = 0;
			double kV = 0;

			public static SimpleMotorFeedforward getFeedforward() {
				return new SimpleMotorFeedforward(kS, kV, kA);
			}

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

	public interface Turn {
		public interface Encoder {
			double GEAR_RATIO = 1.0;

			double GRAYHILL_PULSES_PER_REVOLUTION = 256;
			double GRAYHILL_DISTANCE_PER_PULSE =
					(WHEEL_CIRCUMFERENCE / GRAYHILL_PULSES_PER_REVOLUTION);

			int PORT_A = 0;
			int PORT_B = 1;
		}

		public interface Feedforward {
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
