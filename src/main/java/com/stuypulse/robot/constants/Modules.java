package com.stuypulse.robot.constants;

import com.stuypulse.robot.subsystems.swerve.DriveControl;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Modules {
    double MAX_SPEED = Units.feetToMeters(20.0);

    double WIDTH = Units.feetToMeters(30.0);
    double HEIGHT = Units.feetToMeters(30.0);

    public interface TopRight {
        String ID = "Top Right";
        Translation2d LOCATION = new Translation2d(WIDTH * +0.5, HEIGHT * +0.5);

        public interface Drive {
            SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.02);

            DriveControl CONTROLLER = new DriveControl(FEEDFORWARD, FEEDBACK);
        }

        public interface Turn {
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.05);

            TurnControl CONTROLLER = new TurnControl(FEEDBACK);
        }

        Module MODULE = new Module(ID, LOCATION, Drive.CONTROLLER, Turn.CONTROLLER);
    }

    public interface TopLeft {
        String ID = "Top Left";
        Translation2d LOCATION = new Translation2d(WIDTH * -0.5, HEIGHT * +0.5);

        public interface Drive {
            SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.02);

            DriveControl CONTROLLER = new DriveControl(FEEDFORWARD, FEEDBACK);
        }

        public interface Turn {
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.05);

            TurnControl CONTROLLER = new TurnControl(FEEDBACK);
        }

        Module MODULE = new Module(ID, LOCATION, Drive.CONTROLLER, Turn.CONTROLLER);
    }

    public interface BottomLeft {
        String ID = "Bottom Left";
        Translation2d LOCATION = new Translation2d(WIDTH * -0.5, HEIGHT * -0.5);

        public interface Drive {
            SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.02);

            DriveControl CONTROLLER = new DriveControl(FEEDFORWARD, FEEDBACK);
        }

        public interface Turn {
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.05);

            TurnControl CONTROLLER = new TurnControl(FEEDBACK);
        }

        Module MODULE = new Module(ID, LOCATION, Drive.CONTROLLER, Turn.CONTROLLER);
    }

    public interface BottomRight {
        String ID = "Bottom Right";
        Translation2d LOCATION = new Translation2d(WIDTH * +0.5, HEIGHT * -0.5);

        public interface Drive {
            SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.02);

            DriveControl CONTROLLER = new DriveControl(FEEDFORWARD, FEEDBACK);
        }

        public interface Turn {
            Controller FEEDBACK = new PIDController(0.1, 0.0, 0.05);

            TurnControl CONTROLLER = new TurnControl(FEEDBACK);
        }

        Module MODULE = new Module(ID, LOCATION, Drive.CONTROLLER, Turn.CONTROLLER);
    }

    Module[] MODULES = {
            TopRight.MODULE,
            TopLeft.MODULE,
            BottomLeft.MODULE,
            BottomRight.MODULE
    };
}
