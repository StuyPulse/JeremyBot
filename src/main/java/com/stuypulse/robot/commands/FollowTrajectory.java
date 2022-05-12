package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class FollowTrajectory extends SwerveControllerCommand {

    public FollowTrajectory(Swerve drive, Trajectory trajectory) {
        super(
            trajectory,
            drive::getPose,
            drive.getKinematics(),
            new PIDController(0.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0),
            new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0, 0)),
            drive::setStates,
            drive
        );
    }

    // TODO: add relative methods

}
