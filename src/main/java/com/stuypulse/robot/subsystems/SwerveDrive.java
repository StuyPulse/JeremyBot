package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;
                    
import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.stuylib.math.Vector2D;
                    
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
                    
public class SwerveDrive extends SubsystemBase {
                    
    /** CONSTANTS **/
                    
    private interface Chassis {
        double WIDTH = Units.inchesToMeters(29.0);
        double HEIGHT = Units.inchesToMeters(29.0);
        double MAX_SPEED = Units.feetToMeters(14.0);
    }
    
    private interface FrontRight {
        String ID = "Front Right";
        int DRIVE_PORT = 1;
        int TURN_PORT = 2;
        int ENCODER_PORT = 4;
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(127.65);
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
    }
                    
    private interface FrontLeft {
        String ID = "Front Left";
        int DRIVE_PORT = 3;
        int TURN_PORT = 4;
        int ENCODER_PORT = 3;
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-148.8);
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
    }
    
    private interface BackLeft {
        String ID = "Back Left";
        int DRIVE_PORT = 5;
        int TURN_PORT = 6;
        int ENCODER_PORT = 0;
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-34.2);
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
    }
    
    private interface BackRight {
        String ID = "Back Right";
        int DRIVE_PORT = 7;
        int TURN_PORT = 8;
        int ENCODER_PORT = 2;
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(143.65);
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
    }
                    
    /** MODULES **/
    private final WPI_SwerveModule[] modules;
    
    /** SENSORS  **/
    private final AHRS gyro;
                    
    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
                    
    private final Field2d field;
                    
    public SwerveDrive() {
        
        modules = new WPI_SwerveModule[] {
            new WPI_SwerveModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT, 
                                FrontRight.ENCODER_PORT, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
            new WPI_SwerveModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT, 
                                FrontLeft.ENCODER_PORT, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
            new WPI_SwerveModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT, 
                                BackLeft.ENCODER_PORT, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
            new WPI_SwerveModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT, 
                                BackRight.ENCODER_PORT, BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET)
        };
        
        gyro = new AHRS(SPI.Port.kMXP);
        
        kinematics = new SwerveDriveKinematics(
            getModuleStream()
                .map(x -> x.getModuleOffset())
                .toArray(Translation2d[]::new)
        );
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle());
                    
        field = new Field2d();
        SmartDashboard.putData("Field", field);
                    
        reset(new Pose2d());
    }
                    
    /** MODULE API **/
                    
    public WPI_SwerveModule getModule(String id) {
        for (WPI_SwerveModule module : modules) {
            if (module.getId().equals(id)) 
                return module;
        }
        
        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }
                    
    public WPI_SwerveModule[] getModules() {
        return modules;
    }
                    
    public Stream<WPI_SwerveModule> getModuleStream() {
        return Arrays.stream(getModules());
    }
                    
    public void reset(Pose2d pose) {
        odometry.resetPosition(pose, getGyroAngle()); 
    }
                    
    public void reset() {
        reset(getPose());
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
                    
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Chassis.MAX_SPEED);
        
        for (int i = 0; i < states.length; ++i) {
            modules[i].setTargetState(states[i]);
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
                    