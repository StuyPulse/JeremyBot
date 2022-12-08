package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private SwerveDrive drive;

    private VStream speed;
    private IStream turn;

    public DriveCommand(SwerveDrive drive, Gamepad driver) {
        this.drive = drive;

        speed = VStream.create(driver::getLeftStick).filtered(
            new VDeadZone(0.05),
            new VLowPassFilter(0.1),
            x -> x.mul(4.2)
        );

        turn = IStream.create(driver::getRightX).filtered(
            x -> SLMath.deadband(x, 0.05), 
            x -> x * 6
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
    }

}
