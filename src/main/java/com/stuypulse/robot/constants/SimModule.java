package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
			double GEAR_RATIO = 0.1225;
            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
		}

		public interface Feedforward {
            double kS = 0.11;
            double kV = 2.79;
            double kA = 0.11;

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
			double GEAR_RATIO = 1.0 / 12.8;
            double POSITION_CONVERSION = 2 * Math.PI * GEAR_RATIO;
		}

		public interface Feedforward {
			double kA = 0.01;
			double kV = 0.1;

			public static LinearSystem<N2, N1, N1> getPlant() {
				return LinearSystemId.identifyPositionSystem(kV, kA);
			}
		}

		public interface Feedback {
            SmartNumber kP = new SmartNumber("Turn P", 1.0);
            SmartNumber kI = new SmartNumber("Turn I", 0.0);
            SmartNumber kD = new SmartNumber("Turn D", 0.2);

            public static Controller getController() {
                return new PIDController(kP, kI, kD);
            }
        }
	}
}
