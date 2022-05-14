package com.stuypulse.robot.commands;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Controls;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private Swerve drive;

    private Supplier<Vector2D> speed;
    private IStream turn;

    public DriveCommand(Swerve drive, Gamepad driver) {
        this.drive = drive;
    
        IStream leftX = IStream.create(driver::getLeftX).filtered(Controls.Drive.getFilter());
        IStream leftY = IStream.create(driver::getLeftY).filtered(Controls.Drive.getFilter());

        speed = () -> new Vector2D(leftX.get(), leftY.get()).mul(Controls.MAX_TELEOP_SPEED);

        turn = IStream.create(driver::getRightX)
            .filtered(
                Controls.Turn.getFilter(), 
                x -> x * Controls.MAX_TELEOP_ANGULAR
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
