package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PerfectModule extends SubsystemBase implements SwerveModule {

    /** MODULE **/
    
    private final String id;
    private final Translation2d moduleOffset;
    
    private SwerveModuleState targetState;
    
    public PerfectModule(String id, int turnId, int driveId, int encoderPort, Rotation2d absoluteOffset, Translation2d moduleOffset) {
        // Module
        this.id = id;
        this.moduleOffset = moduleOffset;

        targetState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    }

    /** MODULE METHODS **/
    
    public String getId() {
        return id;
    }

    public Translation2d getModuleOffset() {
        return moduleOffset;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public SwerveModuleState getState() {
        return targetState;
    }

    public double getSpeed() {
        return targetState.speedMetersPerSecond;
    }

    public Rotation2d getAngle() {
        return targetState.angle;
    }
    
    /** CONTROL LOOP **/
    @Override
    public void periodic() {
        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", MathUtil.inputModulus(getAngle().getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Speed", getSpeed());
        
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Speed", targetState.speedMetersPerSecond);
    }

}
