package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.robot.subsystems.modules.Module;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ControlModule extends CommandBase {

    private final Swerve swerve;
    private final Module module;
    private final Gamepad gamepad;

    public ControlModule(Swerve swerve, Module module, Gamepad gamepad) {
        this.swerve = swerve;
        this.module = module; 
        this.gamepad = gamepad;
        addRequirements(swerve);
    }

    public void exeute() {
        for (Module module : swerve.getModules()) {
            if(module == this.module) {
                Vector2D target = gamepad.getLeftStick();
                module.setState(target.magnitude(), target.getAngle().getRotation2d());
            } else {
                module.stop();
            }
        }
    }
    
}
