package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetModuleCommand extends CommandBase {
    
    private Gamepad driver;
    private SwerveModule module;

    public ResetModuleCommand(SwerveDrive drive, SwerveModule module, Gamepad driver) {
        this.driver = driver;
        this.module = module;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        module.reset();
    }

    @Override
    public void execute() {
        Angle stickAngle = driver.getRightStick().getAngle();

        module.setTarget(
            new Polar2D(0.1, stickAngle)
        );
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
