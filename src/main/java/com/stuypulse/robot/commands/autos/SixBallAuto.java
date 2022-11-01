package com.stuypulse.robot.commands.autos;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.PWFollowTrajectory;
import com.stuypulse.robot.util.TrajectoryLoader;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * @author Ivan Chen
 * @author Vincent Wang
 */

public class SixBallAuto extends SequentialCommandGroup {

    private static final double SHOOT_WAIT_TIME = 0.25;
    private static final double TERMINAL_WAIT_TIME = 0.5;

    private static final Trajectory GET_SECOND_BALL = TrajectoryLoader
            .getTrajectory("SixBallAuto/output/GetSecondBall.wpilib.json");
    private static final Trajectory GET_THIRD_BALL = TrajectoryLoader
            .getTrajectory("SixBallAuto/output/GetThirdBall.wpilib.json");
    private static final Trajectory GET_TERMINAL_BALLS = TrajectoryLoader
            .getTrajectory("SixBallAuto/output/GetTerminalBalls.wpilib.json");
    private static final Trajectory TO_SHOOT_TERMINAL_BALLS = TrajectoryLoader
            .getTrajectory("SixBallAuto/output/ToShootTerminalBalls.wpilib.json");
    private static final Trajectory GET_SIXTH_BALL = TrajectoryLoader
            .getTrajectory("SixBallAuto/output/GetSixthBall.wpilib.json");

    public SixBallAuto(RobotContainer robot) {
        addCommands(
                new PWFollowTrajectory(robot.swerve, GET_SECOND_BALL).robotRelative(),
                new WaitCommand(SHOOT_WAIT_TIME),
                new PWFollowTrajectory(robot.swerve, GET_THIRD_BALL).fieldRelative(),
                new WaitCommand(SHOOT_WAIT_TIME),
                new PWFollowTrajectory(robot.swerve, GET_TERMINAL_BALLS).fieldRelative(),
                new WaitCommand(TERMINAL_WAIT_TIME),
                new PWFollowTrajectory(robot.swerve, TO_SHOOT_TERMINAL_BALLS).fieldRelative(),
                new WaitCommand(SHOOT_WAIT_TIME),
                new PWFollowTrajectory(robot.swerve, GET_SIXTH_BALL).fieldRelative(),
                new WaitCommand(SHOOT_WAIT_TIME));
    }
}
