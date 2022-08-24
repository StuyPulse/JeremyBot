package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.Controls.*;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.AStick;
import com.stuypulse.stuylib.streams.angles.AStream;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.vectors.VStream;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SnapDrive extends CommandBase {

    /** CONSTANTS **/

    public interface Turn {
        SmartNumber DEADZONE = new SmartNumber("Controls/Snap/Deadband", 0.5);
    }

    public interface FF {
        double kS = 0.0;
        double kV = 1.0;
        double kA = 0.0;
    }

    public interface Feedback {
        double kP = 10.0;
        double kI = 0.0;
        double kD = 0.1;
    }

    public interface Profile {
        SmartNumber ACCEL_LIMIT = new SmartNumber("Controls/Snap/Accel Limit", 2);
        SmartNumber JERK_LIMIT = new SmartNumber("Controls/Snap/Jerk Limit", 10);
    }


    private final SwerveDrive swerve;

    private final VStream drive;
    private final AStream turn;

    private final AngleController control;

    public SnapDrive(SwerveDrive swerve, Gamepad gamepad) {
        this.swerve = swerve;

        drive = VStream.create(gamepad::getLeftStick).filtered(Drive.getFilter());
        turn = new AStick(gamepad::getRightStick, Turn.DEADZONE);
        
        control = new Feedforward.Motor(FF.kA, FF.kS, FF.kV).angle()
            .setSetpointFilter(new AMotionProfile(Profile.ACCEL_LIMIT, Profile.JERK_LIMIT))
            .add(new AnglePIDController(Feedback.kP, Feedback.kI, Feedback.kD));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        double omega = control.update(turn.get(), Angle.fromRotation2d(swerve.getGyroAngle()));

        swerve.setStates(drive.get(), omega);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {}

}
