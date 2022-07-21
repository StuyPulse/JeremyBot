package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Modules;
import com.stuypulse.robot.constants.Motion;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final Module[] modules;
    private final AHRS gyro;
    private final StopWatch simGyroTimer;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d[] moduleObjects;

    public Swerve() {
        modules = Modules.MODULES;
        gyro = new AHRS(SPI.Port.kMXP);
        simGyroTimer = new StopWatch();

        kinematics = Motion.KINEMATICS;
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle());

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        moduleObjects = new FieldObject2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            moduleObjects[i] = field.getObject(modules[i].getID() + " Module");
        }

        reset(new Pose2d());
    }

    private Translation2d getCenterOfGravity() {
        return new Translation2d(Modules.COG_X.get(), Modules.COG_Y.get());
    }

    /** MODULE API **/

    public Module getModule(String id) {
        for (Module module : modules) {
            if (module.getID().equals(id)) 
                return module;
        }
        
        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public Stream<Module> getModuleStream() {
        return Arrays.stream(getModules());
    }

    public Module[] getModules() {
        return modules;
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(pose, getGyroAngle()); 
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
            setStates(ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x, -omega, getAngle())); // TODO: see TODO above
        } else {
            setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega));
        }
    }

    public void setStates(Vector2D velocity, double omega) {
        setStates(velocity, omega, true);
    }

    public void setStates(ChassisSpeeds robotSpeed) {
        setStates(kinematics.toSwerveModuleStates(robotSpeed, getCenterOfGravity()));
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

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }
    
    /** ODOMETRY API */

    private void updateOdometry() {
        odometry.update(
            getGyroAngle(), 
    
            getModuleStream()
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

    private void updateField() {
        field.setRobotPose(getPose());

        for (int i = 0; i < modules.length; ++i) {
            var module = modules[i];
            var object = moduleObjects[i];

            var rotation = getPose().getRotation();

            object.setPose(new Pose2d(
                getPose().getTranslation().plus(module.getLocation().rotateBy(getPose().getRotation())),
                module.getAngle().plus(rotation)
            ));
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateField();

        SmartDashboard.putNumber("Swerve/Pose X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Angle", gyro.getRotation2d().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        var states = getModuleStream().map(x -> x.getState()).toArray(SwerveModuleState[]::new);
        var speeds = getKinematics().toChassisSpeeds(states);
        
        gyro.setAngleAdjustment(gyro.getAngle() + Math.toDegrees(speeds.omegaRadiansPerSecond * simGyroTimer.reset()));
    }

}