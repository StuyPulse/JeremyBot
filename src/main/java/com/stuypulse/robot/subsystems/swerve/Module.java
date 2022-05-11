package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    private /* final */ Translation2d location;

    private /* final */ DriveControl drive;
    private /* final */ TurnControl turn;

    private SwerveModuleState target;

    public Module(Translation2d location) {
        this.location = location;
        
        // drive = null;
        // turn = null;

        target = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getVelocity(), turn.getAngle());
    }

    public void setState(SwerveModuleState target) {
        this.target = SwerveModuleState.optimize(target, turn.getAngle());
    }

    public Translation2d getLocation() {
        return location;
    }

    @Override
    public void periodic() {
        drive.setVelocity(target.speedMetersPerSecond);
        turn.setAngle(target.angle);
    }

}
