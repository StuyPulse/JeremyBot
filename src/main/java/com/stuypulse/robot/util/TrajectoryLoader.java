/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Motion;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;

public final class TrajectoryLoader {

    public static final TrajectoryConfig MAX_SPEED_CONFIG = 
        new TrajectoryConfig(Motion.MAX_VELOCITY,Motion.MAX_ACCELERATION)
            .setKinematics(Motion.KINEMATICS);

    // Function that gets a trajectory from path weaver,
    // but will give a default one if it has an issue
    public static Trajectory getTrajectory(String path) {
        try {
            return TrajectoryUtil.fromPathweaverJson(Settings.DEPLOY_DIRECTORY.resolve(path));
        } catch (IOException e) {
            DriverStation.reportError("Error Opening \"" + path + "\"!", e.getStackTrace());

            System.err.println("Error Opening \"" + path + "\"!");
            System.out.println(e.getStackTrace());
            System.exit(694);

            return null;
        }
    }

    // Function that gets multiple trajectories and concatinates them together
    public static Trajectory getTrajectory(String... paths) {
        Trajectory trajectory = getTrajectory(paths[0]);

        for (int i = 1; i < paths.length; ++i) {
            trajectory = trajectory.concatenate(getTrajectory(paths[i]));
        }

        return trajectory;
    }
}