package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnModule extends CommandBase {
    
    private final SwerveDrive swerve;
    private final Gamepad gamepad;

    public TurnModule(SwerveDrive swerve, Gamepad gamepad) {
        this.swerve = swerve;
        this.gamepad = gamepad;
        addRequirements(swerve);
    }

    public void execute() {
        for (Module module : swerve.getModules()) {
            module.setState(0.01, gamepad.getRightStick().getAngle().getRotation2d());
        }
    }
}
