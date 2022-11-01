package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.Motion;
import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PWFollowTrajectory extends SwerveControllerCommand {

    private final SwerveDrive swerve;

    private final Trajectory trajectory;
    private boolean robotRelative;

    public PWFollowTrajectory(SwerveDrive swerve, Trajectory trajectory) {
        super(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            Motion.X.getController(),
            Motion.Y.getController(),
            Motion.Theta.getProfileController(),
            swerve::setStates,
            swerve
        );

        this.swerve = swerve;

        this.trajectory = trajectory;
        robotRelative = false;
    }

    public PWFollowTrajectory robotRelative() {
        robotRelative = true;
        return this;
    }

    public PWFollowTrajectory fieldRelative() {
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
