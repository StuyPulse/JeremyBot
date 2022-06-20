package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Controls;
import com.stuypulse.robot.constants.Motion.Theta;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointDrive extends CommandBase {
    private final Swerve swerve;

    private final Translation2d target;
    private final Gamepad gamepad;

    private final StopWatch timer;

    private Rotation2d lastAngle;

    public PointDrive(Gamepad gamepad, Swerve swerve, Translation2d target) {
        this.swerve = swerve;
        
        this.gamepad = gamepad;
        this.target = target;

        timer = new StopWatch();
        lastAngle = new Rotation2d();

        addRequirements(swerve);
    }

    private State getState(double dt) {
        return new State(
            swerve.getAngle().getRadians(),
            (swerve.getAngle().minus(lastAngle)).times(1.0 / dt).getRadians()
        );
    }

    private State getGoal() {
        Vector2D t = new Vector2D(target);
        Vector2D m = new Vector2D(swerve.getPose().getTranslation());
        Angle a = Angle.fromRotation2d(swerve.getPose().getRotation());

        return new State(
            a.sub(t.sub(m).getAngle()).toRadians(),
            0
        );
    }

    private TrapezoidProfile getProfile(double dt) {
        return new TrapezoidProfile(Theta.kConstraints, getGoal(), getState(dt));
    }

    public void initialize() {
        lastAngle = swerve.getAngle();
    }

    public void execute() {
        final double dt = timer.reset();

        Vector2D translation = gamepad.getLeftStick().mul(Controls.MAX_TELEOP_SPEED);
        double omega;

        if (gamepad.getRawRightButton()) {
            omega = getProfile(dt).calculate(dt).velocity;
        } else {
            omega = gamepad.getRightX() * Controls.MAX_TELEOP_SPEED;
        }

        lastAngle = swerve.getAngle();
        swerve.setStates(translation, omega);
    }

    public boolean isFinished() {
        return false;
    }

    public void end() {
        swerve.stop();
    }

}
