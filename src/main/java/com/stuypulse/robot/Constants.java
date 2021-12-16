// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {

    interface Ports {
        int DRIVER = 0;
    }

    interface SwerveDrive {
        interface Ports {
            int TOP_RIGHT_DRIVE = 1;
            int TOP_RIGHT_PIVOT = 2;
        
            int TOP_LEFT_DRIVE = 3;
            int TOP_LEFT_PIVOT = 4;

            int BOTTOM_LEFT_DRIVE = 5;
            int BOTTOM_LEFT_PIVOT = 6;

            int BOTTOM_RIGHT_DRIVE = 7;
            int BOTTOM_RIGHT_PIVOT = 8;
        }

        double TRACK_WIDTH = 0.0;
        double TRACK_HEIGHT = 0.0;

        Vector2D TRACK_SIZE = new Vector2D(TRACK_WIDTH, TRACK_HEIGHT);
    }

    interface SwerveModule {

        // this converts:
        //  - pivot motor revolutions into,
        //  - drive wheel revolutions into,
        //  - drive wheel rotation in radians
        double PIVOT_CONVERSION = 2 * Math.PI / 12.8;

        SmartNumber ANGLE_P = new SmartNumber("Module/AngleP", 0.1);
        SmartNumber ANGLE_I = new SmartNumber("Module/AngleI", 0.0);
        SmartNumber ANGLE_D = new SmartNumber("Module/AngleD", 0.0);

        int SMART_LIMIT = 5; // amps
    }

    interface DriveCommand {
        SmartNumber DRIVE_RC = new SmartNumber("DriveCommand/DriveRC", 0.2);
    }

}
