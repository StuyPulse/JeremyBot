package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetModule extends CommandBase {
    
    private Gamepad driver;
    private Module module;

    public ResetModule(Swerve drive, Module module, Gamepad driver) {
        this.driver = driver;
        this.module = module;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // module.reset();
    }

    @Override
    public void execute() {
        Angle stickAngle = driver.getRightStick().getAngle();
        module.setState(0.1, stickAngle.getRotation2d());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {
        // module.reset();
    }

}
