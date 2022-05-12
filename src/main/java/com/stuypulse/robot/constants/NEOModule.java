package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.util.NEOConfig;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
            double kS = 0.0;
            double kV = 12.0 / MAX_SPEED;
            double kA = 0.01;

            public static SimpleMotorFeedforward getFeedforward() {
                return new SimpleMotorFeedforward(kS, kV, kA);
            }
        }

        public interface Feedback {
            double kP = 0.1;
            double kI = 0.0;
            double kD = 0.0;

            public static Controller getController() {
                return new PIDController(kP, kI, kD);
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
            double kP = 0.01;
            double kI = 0.0;
            double kD = 0.005;

            public static Controller getController() {
                return new PIDController(kP, kI, kD);
            }
        }
    }

}
