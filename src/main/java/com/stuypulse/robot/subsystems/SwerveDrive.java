package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.Derivative;
import com.stuypulse.stuylib.util.AngleVelocity;

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
    private final SL_SimModule[] modules;
    
    /** SENSORS  **/
    private final AHRS gyro;

    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
                    
    private final Field2d field;

    public SwerveDrive() {
        
        modules = new SL_SimModule[] {
            new SL_SimModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT, 
                                FrontRight.ENCODER_PORT, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
            new SL_SimModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT, 
                                FrontLeft.ENCODER_PORT, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
            new SL_SimModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT, 
                                BackLeft.ENCODER_PORT, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
            new SL_SimModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT, 
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

    public SL_SimModule getModule(String id) {
        for (SL_SimModule module : modules) {
            if (module.getId().equals(id)) 
                return module;
        }

        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public SL_SimModule[] getModules() {
        return modules;
    }

    public Stream<SL_SimModule> getModuleStream() {
        return Arrays.stream(getModules());
    }

    public SwerveModuleState[] getModuleStates() {
        return getModuleStream().map(x -> x.getState()).toArray(SwerveModuleState[]::new);
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(pose, getGyroAngle()); 
    }

    public void reset() {
        reset(getPose());
    }

    /** MODULE STATES API **/

    private static double getMaxSpeed(SwerveModuleState[] states) {
        double m = Chassis.MAX_SPEED;
        for (var state : states) {
            if (state.speedMetersPerSecond > m) m = state.speedMetersPerSecond;
        }
        return m;
    }

    private static Vector2D getVector(ChassisSpeeds state) {
        // return new Vector2D(-state.vyMetersPerSecond, state.vxMetersPerSecond);
        return new Vector2D(state.vxMetersPerSecond, state.vyMetersPerSecond);
    }

    private SmartNumber pooballs = new SmartNumber("Swerve/Poo Balls", 8);

    public void setStates(Vector2D velocity, double omega, boolean fieldRelative) {
        if (fieldRelative) {
            // try correcting turn angle
            var rawSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x, -omega, getAngle());
            for (int i = 0; i < 8; ++i) {
                var rawStates = kinematics.toSwerveModuleStates(rawSpeeds);
                double maxSpeed = getMaxSpeed(rawStates);
                double correctionRatio = Chassis.MAX_SPEED / maxSpeed;
                
                // var realSpeed = kinematics.toChassisSpeeds(getModuleStates());
                // Vector2D realVel = new Vector2D(realSpeed.vxMetersPerSecond, realSpeed.vyMetersPerSecond);
                // Vector2D satVel = 

                // SwerveDriveKinematics.desaturateWheelSpeeds(rawStates, Chassis.MAX_SPEED);
                // var targetSpeeds = kinematics.toChassisSpeeds(rawStates);
                // var realSpeeds = kinematics.toChassisSpeeds(getModuleStates());

                // double offsetRadians = new Vector2D(velocity.y, -velocity.x).getAngle()
                //             .sub(getVector(realSpeeds).getAngle().sub(Angle.fromRotation2d(getAngle()))).toRadians();
                //                 // getVector(realSpeeds).rotate(Angle.fromRotation2d(getAngle()).negative()).normalize());
                // // double offsetAngle = Math.asin(offset);

                // if (getVector(realSpeeds).magnitude() < 1) offsetRadians = 0;

                // System.out.println(i + ": " + correctionRatio);

                var correction = new Rotation2d(0.5 * omega * Settings.dT * correctionRatio);
                rawSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x, -omega, getAngle().plus(correction));
            }

            // System.out.println(correctionRatio);
            setStates(rawSpeeds);
        } else {
            setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega));
        }
    }

    AngleVelocity angularVel = new AngleVelocity();
    public double getAngularVelocity() {
        return angularVel.get(Angle.fromRotation2d(getAngle()));
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
        odometry.update(getGyroAngle(), getModuleStates());
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

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() + Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.dT));
        // gyro.setAngleAdjustment(getPose().getRotation().getDegrees());
    }

}