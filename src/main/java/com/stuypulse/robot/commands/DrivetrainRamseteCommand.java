package com.stuypulse.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

import com.stuypulse.robot.Constants.Motion;
import com.stuypulse.robot.Constants.Odometry;
import com.stuypulse.robot.Constants.SwerveModule;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.robot.subsystems.SwerveDrive;

public class DrivetrainRamseteCommand extends SwerveControllerCommand {
    public DrivetrainRamseteCommand(SwerveDrive drivetrain, Trajectory trajectory) {
        super(
                trajectory,
                Odometry.ODOMETRY.getPoseMeters(),
                Motion.KINEMATICS,
                new PIDController(Motion.X_PID.kP, Motion.X_PID.kI, Motion.X_PID.kD),
                new PIDController(Motion.Y_PID.kP, Motion.Y_PID.kI, Motion.Y_PID.kD),
                new ProfiledPIDController(SwerveModule.ANGLE_P.doubleValue(), SwerveModule.ANGLE_I.doubleValue(), SwerveModule.ANGLE_D.doubleValue(), new Constraints(Motion.MAX_VELOCITY, Motion.MAX_ACCELERATION)),
                x,
                drivetrain);
    }
}