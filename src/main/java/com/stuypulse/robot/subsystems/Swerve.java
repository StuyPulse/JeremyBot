package com.stuypulse.robot.subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Modules;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final Module[] modules;
    private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;

    public Swerve() {
        modules = Modules.MODULES;
        gyro = new AHRS(SPI.Port.kMXP);
        
        kinematics = new SwerveDriveKinematics(
            Arrays.stream(modules)
                .map(x -> x.getLocation())
                .toArray(Translation2d[]::new)
        );
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

        field = new Field2d();
    }

    /** MODULE API **/

    public Module getModule(String id) {
        for (Module module : modules) {
            if (module.getID().equals(id)) 
                return module;
        }
        
        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public Module[] getModules() {
        return modules;
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(pose, getAngle());
        // gyro.reset();
        for (Module module : modules) {
            module.reset();
        }
    }

    public void reset() {
        reset(getPose());
    }

    public void stop() {
        for (Module module : modules) {
            module.stop();
        }
    }

    /** MODULE STATES API **/

    public void setStates(Vector2D velocity, double omega, boolean fieldRelative) {
        if (fieldRelative) {
            setStates(ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x, -omega, getAngle()));
        } else {
            setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega));
        }
    }

    public void setStates(Vector2D velocity, double omega) {
        setStates(velocity, omega, true);
    }

    public void setStates(ChassisSpeeds robotSpeed) {
        setStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    public void setStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Modules.MAX_SPEED);
        
        for (int i = 0; i < states.length; ++i) {
            modules[i].setState(states[i]);
        }
    }

    /** GYRO API */
    
    /** ODOMETRY API */

    private void updateOdometry() {
        odometry.update(
            gyro.getRotation2d(), 
    
            Arrays.stream(modules)
                .map(x -> x.getState())
                .toArray(SwerveModuleState[]::new)
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());
       
        SmartDashboard.putNumber("Swerve/Pose X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Angle", gyro.getRotation2d().getDegrees());
    }

}
