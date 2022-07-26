package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.WPI_SwerveModule;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        for (WPI_SwerveModule module : swerve.getModules()) {
            module.setTargetState(new SwerveModuleState(0.01, gamepad.getRightStick().getAngle().getRotation2d()));
        }
    }
}
