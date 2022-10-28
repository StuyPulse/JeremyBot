package com.stuypulse.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.stuypulse.robot.constants.Settings.Motion;
import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTrajectory extends PPSwerveControllerCommand {

    private final SwerveDrive swerve;

    private final Trajectory trajectory;
    private boolean robotRelative;

    public FollowTrajectory(SwerveDrive swerve, PathPlannerTrajectory trajectory, HashMap<String, Command> eventMap) {
        super(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            Motion.X.getController(),
            Motion.Y.getController(),
            Motion.Theta.getController(),
            swerve::setStates,
            eventMap,
            swerve
        );

        this.swerve = swerve;

        this.trajectory = trajectory;
        robotRelative = false;
    }
    
    public FollowTrajectory(SwerveDrive swerve, PathPlannerTrajectory trajectory) {
        super(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            Motion.X.getController(),
            Motion.Y.getController(),
            Motion.Theta.getController(),
            swerve::setStates,
            swerve
        );

        this.swerve = swerve;

        this.trajectory = trajectory;
        robotRelative = false;
    }

    public FollowTrajectory(SwerveDrive swerve, String path, PathConstraints constraints, HashMap<String, Command> eventMap) {
        this(swerve, PathPlanner.loadPath(path, constraints), eventMap);
    }
    
    public FollowTrajectory(SwerveDrive swerve, String path, PathConstraints constraints) {
        this(swerve, PathPlanner.loadPath(path, constraints));
    }

    public FollowTrajectory(SwerveDrive swerve, String path, HashMap<String, Command> eventMap) {
        this(swerve, path, Motion.DEFAULT_CONSTRAINTS, eventMap);
    }

    public FollowTrajectory(SwerveDrive swerve, String path) {
        this(swerve, path, Motion.DEFAULT_CONSTRAINTS);
    }

    public FollowTrajectory robotRelative() {
        robotRelative = true;
        return this;
    }

    public FollowTrajectory fieldRelative() {
        robotRelative = false;
        return this;
    }

    public void initialize() {
        if (robotRelative) {
            swerve.reset(trajectory.getInitialPose());
        }

        super.initialize();
    }

}
