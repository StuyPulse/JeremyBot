package com.stuypulse.robot.commands;

import java.util.function.Supplier;

import com.stuypulse.robot.Constants.Controls;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private Swerve drive;

    private Supplier<Vector2D> speed;
    private IStream turn;

    public DriveCommand(Swerve drive, Gamepad driver) {
        this.drive = drive;
    
        speed = () -> driver.getLeftStick().mul(Units.feetToMeters(17.0));

        turn = IStream.create(() -> driver.getRightTrigger() - driver.getLeftTrigger())
            .filtered(new LowPassFilter(Controls.TURN_RC));

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
    }

}
