package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    private final String id;

    private final Translation2d location;

    private final DriveControl drive;
    private final TurnControl turn;

    private SwerveModuleState target;

    public Module(String id, Translation2d location, DriveControl drive, TurnControl turn) {
        this.id = id;
        this.location = location;
        
        this.drive = drive;
        this.turn = turn;

        target = new SwerveModuleState(0.0, new Rotation2d(0.0));

        // setup shuffleboard
        ShuffleboardTab me = Shuffleboard.getTab(id);        
        me.addNumber("Target Velocity (m/s) ", () -> target.speedMetersPerSecond);
        me.addNumber("Velocity (m/s)", () -> drive.getVelocity());
        me.addNumber("Target Angle (deg)", () -> target.angle.getDegrees());
        me.addNumber("Angle (deg)", () -> turn.getAngle().getDegrees());
    }

    public String getID() {
        return id;
    }

    public Translation2d getLocation() {
        return location;
    }

    /** SWERVE STATE API **/

    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getVelocity(), turn.getAngle());
    }

    public void setState(SwerveModuleState target) {
        this.target = SwerveModuleState.optimize(target, turn.getAngle());
    }

    public void setState(double velocity, Rotation2d angle) {
        setState(new SwerveModuleState(velocity, angle));
    }

    @Override
    public void periodic() {
        drive.setVelocity(target.speedMetersPerSecond);
        turn.setAngle(target.angle);

        // Shuffleboard.getTab(id)
        //     .addNumber("Target Velocity (m/s) ", () -> target.speedMetersPerSecond))
        //     .addNumber("Velocity (m/s)", () -> drive.getVelocity())
        //     .addNumber("Target Angle (deg)", () -> target.angle.getDegrees())
        //     .addNumber("Angle (deg)", turn.getAngle().getDegrees());

        // SmartDashboard.putNumber(id + "/Target Velocity (m/s)", target.speedMetersPerSecond);
        // SmartDashboard.putNumber(id + "/Target Angle (deg)", target.angle.getDegrees());

        // SmartDashboard.putNumber(id + "/Velocity (m/s)", drive.getVelocity());
        // SmartDashboard.putNumber(id + "/Angle (deg)", turn.getAngle().getDegrees());
    }

}
