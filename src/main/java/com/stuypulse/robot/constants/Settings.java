package com.stuypulse.robot.constants;

import java.nio.file.Path;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public interface Settings {
    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
    int UPDATE_RATE = 50;
    double dT = 1.0 / UPDATE_RATE;

    public interface Robot {
        public interface Control {
            public interface Turn {
                SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
                SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
                SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.1);

                double kS = 0.14;
                double kV = 0.25;
                double kA = 0.007;
            }

            SimpleMotorFeedforward poop = new SimpleMotorFeedforward(Drive.kS.get(), Drive.kV.get(), Drive.kA.get());
        
            public interface Drive {
                SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 3);
                SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0);
                SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0);
        
                SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.17335);
                SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.7274);
                SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.456);
            }
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    
                public interface Stages {
                    // input / output
                    double FIRST = 16.0 / 48.0;
                    double SECOND = 28.0 / 16.0;
                    double THIRD = 15.0 / 60.0;
                }
    
                double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;
    
                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
    
            public interface Turn {
                double GEAR_RATIO = 1.0 / 12.8;
                double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
    }

    public interface Motion {

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
                        new Constraints(3, 3)); // TODO: fill in these values
            }
        }
    }
}
