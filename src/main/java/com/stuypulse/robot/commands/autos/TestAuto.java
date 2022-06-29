package com.stuypulse.robot.commands.autos;

import java.util.List;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.FollowTrajectory;
import com.stuypulse.robot.util.TrajectoryLoader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup {
    
    private static final Trajectory TRAJECTORY = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.0, 0.0, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(5, -0.5, new Rotation2d(0.0)),
            TrajectoryLoader.MAX_SPEED_CONFIG
        );

    public TestAuto(RobotContainer robot) {
        addCommands(new FollowTrajectory(robot.swerve, TRAJECTORY).robotRelative());
    }

}
