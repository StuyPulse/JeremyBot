package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.util.NEOConfig;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * This file acts as a data sheet for our MK3 Swerve Module with
 * NEO550's and CANSparkMaxes. 
 * 
 * It contains general information like max speed, wheel size, and gear
 * ratios, but it also contains our Feedforward and PID constants we've
 * found for these modules.
 * 
 * The specific settings we want for the CANSparkMaxes is also contained here,
 * which includes the gearing ratio and current limit.
 *
 * @author Myles Pasetsky (myles.pasetsky@gmail.com) 
 */
public interface NEOModule {
    // https://www.swervedrivespecialties.com/products/mk3-swerve-module?variant=31575980703857
    
    double MAX_SPEED = Units.feetToMeters(14.4);

    double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public interface Drive {
        int CURRENT_LIMIT = 80;

        public interface Encoder {
            public interface Stages {
                // input / output 
                double FIRST = 14.0 / 50.0;
                double SECOND = 28.0 / 16.0;
                double THIRD = 15.0 / 60.0;
            }

            double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }

        public static NEOConfig getConfig() {
            return new NEOConfig()
                .setIdleMode(IdleMode.kCoast)
                .setSmartCurrentLimit(CURRENT_LIMIT)
                .setPositionConversion(Encoder.POSITION_CONVERSION)
                .setVelocityConversion(Encoder.VELOCITY_CONVERSION);
        }

        public interface Feedforward {
            double kS = 0.11004;
            double kV = 2.7859;
            double kA = 0.10702;

            public static SimpleMotorFeedforward getFeedforward() {
                return new SimpleMotorFeedforward(kS, kV, kA);
            }
        }

        public interface Feedback {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.0;

            public static Controller getController() {
                return new PIDController(kP, kI, kD);
            }
        }

        public interface StateSpace {
            double STATE_STDEV = 3.0;
            double MEASURE_STDEV = 0.01;

            double Q_ERROR_TOLERANCE = 4.0;
            double R_CONTROL_TOLERANCE = 12.0;

            double MAX_VOLTS = 12.0;

            public static LinearSystemLoop<N1, N1, N1> getControlLoop() {
                LinearSystem<N1, N1, N1> model = LinearSystemId.identifyVelocitySystem(Feedforward.kV, Feedforward.kA);
                KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                    Nat.N1(), 
                    Nat.N1(), 
                    model, 
                    VecBuilder.fill(STATE_STDEV), 
                    VecBuilder.fill(MEASURE_STDEV),
                    Motion.dt
                );
                LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
                    model,
                    VecBuilder.fill(Q_ERROR_TOLERANCE),
                    VecBuilder.fill(R_CONTROL_TOLERANCE),
                    Motion.dt
                );
                return new LinearSystemLoop<>(
                    model,
                    controller,
                    observer,
                    MAX_VOLTS,
                    Motion.dt
                );
            }
        }
    }

    public interface Turn {
        int CURRENT_LIMIT = 20;

        public interface Encoder {
            double GEAR_RATIO = 1.0 / 12.8;
            double POSITION_CONVERSION = 2 * Math.PI * GEAR_RATIO;
        }

        public static NEOConfig getConfig() {
            return new NEOConfig()
                .setIdleMode(IdleMode.kBrake)
                .setSmartCurrentLimit(CURRENT_LIMIT)
                .setPositionConversion(Encoder.POSITION_CONVERSION);
        }

        public interface Feedback {
            SmartNumber kP = new SmartNumber("Turn P", 0.1);
            SmartNumber kI = new SmartNumber("Turn I", 0.0);
            SmartNumber kD = new SmartNumber("Turn D", 0.0);

            public static Controller getController() {
                return new PIDController(kP, kI, kD);
            }
        }
    }

}
