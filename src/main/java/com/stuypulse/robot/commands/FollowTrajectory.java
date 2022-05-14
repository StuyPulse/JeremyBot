package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Motion;
import com.stuypulse.robot.subsystems.Swerve;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class FollowTrajectory extends SwerveControllerCommand {

    public FollowTrajectory(Swerve drive, Trajectory trajectory) {
        super(
            trajectory,
            drive::getPose,
            drive.getKinematics(),
            Motion.X.getController(),
            Motion.Y.getController(),
            Motion.Theta.getController(),
            drive::setStates,
            drive
        );
    }

    // TODO: add relative methods

}
