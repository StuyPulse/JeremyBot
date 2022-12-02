package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.DriverSettings.*;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.vectors.VStream;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private SwerveDrive drive;

    private VStream speed;
    private IStream turn;

    public DriveCommand(SwerveDrive drive, Gamepad driver) {
        this.drive = drive;

        speed = VStream.create(driver::getLeftStick).filtered(Drive.getFilter());
        turn = IStream.create(driver::getRightX).filtered(Turn.getFilter());

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
