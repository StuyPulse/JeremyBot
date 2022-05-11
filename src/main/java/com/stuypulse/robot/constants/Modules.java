package com.stuypulse.robot.constants;

import com.stuypulse.robot.subsystems.swerve.Module;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Modules {
    double WIDTH = Units.feetToMeters(30.0);
    double HEIGHT = Units.feetToMeters(30.0);

    public interface TopRight {
        String ID = "Top Right";
        Translation2d LOCATION = new Translation2d(WIDTH * +0.5, HEIGHT * +0.5);

        Module MODULE = new Module(ID, LOCATION);
    }

    public interface TopLeft {
        String ID = "Top Left";
        Translation2d LOCATION = new Translation2d(WIDTH * -0.5, HEIGHT * +0.5);

        Module MODULE = new Module(ID, LOCATION);
    }

    public interface BottomLeft {
        String ID = "Bottom Left";
        Translation2d LOCATION = new Translation2d(WIDTH * -0.5, HEIGHT * -0.5);

        Module MODULE = new Module(ID, LOCATION);
    }

    public interface BottomRight {
        String ID = "Bottom Right";
        Translation2d LOCATION = new Translation2d(WIDTH * +0.5, HEIGHT * -0.5);

        Module MODULE = new Module(ID, LOCATION);
    }

    Module[] MODULES = {
            TopRight.MODULE,
            TopLeft.MODULE,
            BottomLeft.MODULE,
            BottomRight.MODULE
    };
}
