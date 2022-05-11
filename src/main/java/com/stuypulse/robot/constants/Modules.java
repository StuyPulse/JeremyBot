package com.stuypulse.robot.constants;

import com.stuypulse.robot.subsystems.swerve.Module;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Modules {
    double WIDTH = Units.feetToMeters(30.0);
    double HEIGHT = Units.feetToMeters(30.0);

    public interface TopRight {
        Translation2d LOCATION = new Translation2d(WIDTH * +0.5, HEIGHT * +0.5);

        Module MODULE = new Module(LOCATION);
    }

    public interface TopLeft {
        Translation2d LOCATION = new Translation2d(WIDTH * -0.5, HEIGHT * +0.5);

        Module MODULE = new Module(LOCATION);
    }

    public interface BottomLeft {
        Translation2d LOCATION = new Translation2d(WIDTH * -0.5, HEIGHT * -0.5);

        Module MODULE = new Module(LOCATION);
    }

    public interface BottomRight {
        Translation2d LOCATION = new Translation2d(WIDTH * +0.5, HEIGHT * -0.5);

        Module MODULE = new Module(LOCATION);
    }

    Module[] MODULES = {
            TopRight.MODULE,
            TopLeft.MODULE,
            BottomLeft.MODULE,
            BottomRight.MODULE
    };
}
