package com.stuypulse.robot.commands.autos;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.stuypulse.robot.Constants.DriveCommand.*;

public class DriveCommand extends CommandBase {
    
    private Gamepad driver;
    private SwerveDrive drivetrain;

    private IFilter turnFilter;

    public DriveCommand(SwerveDrive drivetrain, Gamepad driver) {
        this.driver = driver;
        this.drivetrain = drivetrain;
    
        turnFilter = new LowPassFilter(DRIVE_RC);

        addRequirements(drivetrain);
    }

    private double getRawTurn() {
        return driver.getRightX();
    }

    private double getTurn() {
        return turnFilter.get(getRawTurn());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drivetrain.drive(
            driver.getLeftStick(),
            getTurn()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {
    }

}
