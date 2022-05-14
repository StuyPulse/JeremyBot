package com.stuypulse.robot.commands;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Controls;
import com.stuypulse.robot.constants.Modules;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private Swerve drive;

    private Supplier<Vector2D> speed;
    private IStream turn;

    public DriveCommand(Swerve drive, Gamepad driver) {
        this.drive = drive;
    
        speed = () -> new Vector2D(
                    SLMath.deadband(driver.getLeftX(), Controls.SPEED_DEADBAND),
                    SLMath.deadband(driver.getLeftY(), Controls.SPEED_DEADBAND)
                ).mul(Modules.MAX_SPEED);

        turn = IStream.create(() -> driver.getRightStick().x)
            .filtered(
                x -> SLMath.deadband(x, Controls.TURN_DEADBAND),
                new LowPassFilter(Controls.OMEGA_RC),
                x -> x * Modules.MAX_ANGULAR_SPEED
            );

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.setStates(speed.get(), turn.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {
        drive.stop();
    }

}
