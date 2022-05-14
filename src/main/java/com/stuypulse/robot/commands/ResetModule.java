package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetModule extends CommandBase {
    
    private Swerve drive;
    private Gamepad driver;
    private Module module;

    public ResetModule(Swerve drive, Module module, Gamepad driver) {
        this.driver = driver;
        this.module = module;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        module.reset();
    }

    @Override
    public void execute() {
        Angle stickAngle = driver.getRightStick().getAngle();

        for (Module module : drive.getModules()) {
            if (module == this.module) {
                module.setState(0.0, stickAngle.getRotation2d());
            } else {
                module.stop();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {
        module.reset();
    }

}
