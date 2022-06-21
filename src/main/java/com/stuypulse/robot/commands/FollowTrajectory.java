package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Motion;
import com.stuypulse.robot.subsystems.Swerve;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class FollowTrajectory extends SwerveControllerCommand {

    private final Swerve swerve;

    private final Trajectory trajectory;
    private boolean robotRelative;

    public FollowTrajectory(Swerve swerve, Trajectory trajectory) {
        super(
                trajectory,
                swerve::getPose,
                swerve.getKinematics(),
                Motion.X.getController(),
                Motion.Y.getController(),
                Motion.Theta.getController(),
                swerve::setStates,
                swerve);

        this.swerve = swerve;

        this.trajectory = trajectory;
        robotRelative = false;
    }

    public FollowTrajectory robotRelative() {
        robotRelative = true;
        return this;
    }

    public FollowTrajectory fieldRelative() {
        robotRelative = false;
        return this;
    }

    public void initialze() {
        if (robotRelative) {
            swerve.reset(trajectory.getInitialPose());
        }

        super.initialize();
    }

}
