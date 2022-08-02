package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.*;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ControlModule extends CommandBase {

    private final SwerveDrive swerve;
    private final SL_SimModule module;
    private final Gamepad gamepad;

    public ControlModule(SwerveDrive swerve, SL_SimModule module, Gamepad gamepad) {
        this.swerve = swerve;
        this.module = module; 
        this.gamepad = gamepad;
        addRequirements(swerve);
    }

    public void exeute() {
        for (var module : swerve.getModules()) {
            if(module == this.module) {
                Vector2D target = gamepad.getLeftStick();
                module.setTargetState(new SwerveModuleState(target.magnitude(), target.getAngle().getRotation2d()));
            } else {
                module.setTargetState(new SwerveModuleState(0.0, new Rotation2d(0)));
            }
        }
    }
    
}
