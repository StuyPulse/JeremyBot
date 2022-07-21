package com.stuypulse.robot.commands.autos;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.FollowTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import com.stuypulse.robot.util.TrajectoryLoader;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * @author Ivan Chen
 * @author Vincent Wang
 */

public class FiveBallAuto extends SequentialCommandGroup {

    private static final double SHOOT_WAIT_TIME = 0.25;
    private static final double TERMINAL_WAIT_TIME = 0.5;

    private static final Trajectory GET_SECOND_BALL = TrajectoryLoader
            .getTrajectory("FiveBallAuto/output/GetSecondBall.wpilib.json");
    private static final Trajectory GET_THIRD_BALL = TrajectoryLoader
            .getTrajectory("FiveBallAuto/output/GetThirdBall.wpilib.json");
    private static final Trajectory GET_TERMINAL_BALLS = TrajectoryLoader
            .getTrajectory("FiveBallAuto/output/GetTerminalBalls.wpilib.json");
    private static final Trajectory TO_SHOOT_TERMINAL_BALLS = TrajectoryLoader
            .getTrajectory("FiveBallAuto/output/ToShootTerminalBalls.wpilib.json");

    public FiveBallAuto(RobotContainer robot) {
        addCommands(
                new FollowTrajectory(robot.swerve, GET_SECOND_BALL).robotRelative(),
                new WaitCommand(SHOOT_WAIT_TIME),
                new FollowTrajectory(robot.swerve, GET_THIRD_BALL).fieldRelative(),
                new WaitCommand(SHOOT_WAIT_TIME),
                new FollowTrajectory(robot.swerve, GET_TERMINAL_BALLS).fieldRelative(),
                new WaitCommand(TERMINAL_WAIT_TIME),
                new FollowTrajectory(robot.swerve, TO_SHOOT_TERMINAL_BALLS).fieldRelative(),
                new WaitCommand(SHOOT_WAIT_TIME));
    }
}
