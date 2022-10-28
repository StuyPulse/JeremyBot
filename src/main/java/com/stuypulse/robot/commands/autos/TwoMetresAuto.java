package com.stuypulse.robot.commands.autos;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.FollowTrajectory;
import com.stuypulse.robot.util.TrajectoryLoader;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * @author Ivan Chen
 * @author Vincent Wang
 */

public class TwoMetresAuto extends SequentialCommandGroup {

    private static final Trajectory TO_INFINITY_AND_BEYOND = TrajectoryLoader
            .getTrajectory("TwoMetresAuto/output/DriveForward.wpilib.json");

    public TwoMetresAuto(RobotContainer robot) {
        // addCommands(
        //         new FollowTrajectory(robot.swerve, TO_INFINITY_AND_BEYOND).robotRelative());
    }
}
