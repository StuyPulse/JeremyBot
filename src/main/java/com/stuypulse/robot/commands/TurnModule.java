package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.robot.subsystems.modules.Module;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnModule extends CommandBase {
    
    private final Swerve swerve;
    private final Gamepad gamepad;

    public TurnModule(Swerve swerve, Gamepad gamepad) {
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
