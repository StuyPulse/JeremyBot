package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Controls;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.vectors.VStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private Swerve drive;

    private VStream speed;
    private IStream turn;

    public DriveCommand(Swerve drive, Gamepad driver) {
        this.drive = drive;

        speed = VStream.create(driver::getLeftStick).filtered(Controls.Drive.getFilter());
        turn = IStream.create(driver::getRightX).filtered(Controls.Turn.getFilter());

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var nextSpeed = speed.get();
        var nextTurn = turn.get();
        
        SmartDashboard.putNumber("Controls/Speed (x)", nextSpeed.x);
        SmartDashboard.putNumber("Controls/Speed (y)", nextSpeed.y);

        SmartDashboard.putNumber("Controls/Turn",nextTurn);

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
