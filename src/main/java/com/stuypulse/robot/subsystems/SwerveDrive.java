package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.modules.SL_SimModule;
// import com.stuypulse.robot.subsystems.modules.SL_SimModule;
import com.stuypulse.robot.subsystems.modules.SL_SwerveModule;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.angles.AStream;
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
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    /** CONSTANTS **/

    private interface Chassis {
        double WIDTH = Units.inchesToMeters(29.0);
        double HEIGHT = Units.inchesToMeters(29.0);
        double MAX_SPEED = 4.2;
    }

    private interface FrontRight {
        String ID = "Front Right";
        int DRIVE_PORT = 10;
        int TURN_PORT = 11;
        int ENCODER_PORT = 1;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(143));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
    }

    private interface FrontLeft {
        String ID = "Front Left";
        int DRIVE_PORT = 12;
        int TURN_PORT = 13;
        int ENCODER_PORT = 3;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(36));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
    }

    private interface BackLeft {
        String ID = "Back Left";
        int DRIVE_PORT = 14;
        int TURN_PORT = 15;
        int ENCODER_PORT = 2;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(-80.5));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
    }

    private interface BackRight {
        String ID = "Back Right";
        int DRIVE_PORT = 16;
        int TURN_PORT = 17;
        int ENCODER_PORT = 0;
        SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(142.3));
        Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
    }

    private static SwerveModule makeModule(String id, int turnId, int driveId, int encoderPort,
            SmartAngle absoluteOffset, Translation2d moduleOffset) {
        return new SL_SwerveModule(id, moduleOffset, turnId, encoderPort, absoluteOffset, driveId);
        // return new SL_SimModule(id, moduleOffset);
    }

    /** MODULES **/
    private final SwerveModule[] modules;

    /** SENSORS **/
    private final AHRS gyro;

    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d[] module2ds;


    public SwerveDrive() {

        modules = new SwerveModule[] {
            makeModule(FrontRight.ID, FrontRight.TURN_PORT, FrontRight.DRIVE_PORT,
                FrontRight.ENCODER_PORT, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET),
            makeModule(FrontLeft.ID, FrontLeft.TURN_PORT, FrontLeft.DRIVE_PORT,
                    FrontLeft.ENCODER_PORT, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET),
            makeModule(BackLeft.ID, BackLeft.TURN_PORT, BackLeft.DRIVE_PORT,
                    BackLeft.ENCODER_PORT, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET),
            makeModule(BackRight.ID, BackRight.TURN_PORT, BackRight.DRIVE_PORT,
                    BackRight.ENCODER_PORT, BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET)
        };

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(
                getModuleStream()
                        .map(x -> x.getLocation())
                        .toArray(Translation2d[]::new));
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle());

        field = new Field2d();
        module2ds = new FieldObject2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i] = field.getObject(modules[i].getId()+"-2d");
        }

        SmartDashboard.putData("Field", field);

        reset(new Pose2d());
    }

    public double getVelocity() {
        var cs = kinematics.toChassisSpeeds(getModuleStates());
        return Math.hypot(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    /** MODULE API **/

    public SwerveModule getModule(String id) {
        for (SwerveModule module : modules) {
            if (module.getId().equals(id))
                return module;
        }

        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Stream<SwerveModule> getModuleStream() {
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

    private static double getSaturation(SwerveModuleState[] states) {
        double sat = 1;
        for (var state : states) {
            sat = Math.max(sat, state.speedMetersPerSecond / Chassis.MAX_SPEED);
        }
        return sat;
    }

    public void setStates(Vector2D velocity, double omega, boolean fieldRelative) {
        if (fieldRelative) {
            final Rotation2d correction = new Rotation2d(0.5 * omega * Settings.dT);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y,
                    -velocity.x, -omega,
                    getAngle().plus(correction));

            for (int i = 0; i < 8; ++i) {
                double saturation = getSaturation(kinematics.toSwerveModuleStates(speeds));

                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x,
                        -omega,
                        getAngle().plus(correction.times(1.0 / saturation)));
            }

            setStatesRetainAngle(speeds);
        } else {
            setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega));
        }
    }

    private void setStatesRetainAngle(ChassisSpeeds robotSpeed) {
        var moduleStates = kinematics.toSwerveModuleStates(robotSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Chassis.MAX_SPEED);
        for (int i = 0; i < modules.length; ++i) {
            var currentState = modules[i].getState();
            if (moduleStates[i].speedMetersPerSecond < 0.1) {
                modules[i].setTargetState(new SwerveModuleState(
                    moduleStates[i].speedMetersPerSecond, 
                    currentState.angle
                ));
            } else {
                modules[i].setTargetState(moduleStates[i]);
            }
        }
        // setStates(robotSpeed);
    }

    public void setStates(Vector2D velocity, double omega) {
        setStates(velocity, omega, true);
    }

    public void setStates(ChassisSpeeds robotSpeed) {
        setStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    public void setStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                    "Number of desired module states does not match number of modules (" + modules.length + ")");
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

    // AngleVelocity anglevelocity = new AngleVelocity();

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        var pose = getPose();
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getLocation().rotateBy(getAngle())),
                modules[i].getState().angle.plus(getAngle())
            ));
        }

        // SmartDashboard.putNumber("Swerve/Velocity", getVelocity());
        // SmartDashboard.putNumber("Swerve/")

        // TODO: this is not reporting a sensible value
        // System.out.println(getKinematics().toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);

        // System.out.println(anglevelocity.get(Angle.fromRotation2d(gyro.getRotation2d())));
        SmartDashboard.putNumber("Swerve/Pose X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Angle", gyro.getRotation2d().getDegrees());
        // System.out.println(gyro.getVel);
    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.dT));
        // gyro.setAngleAdjustment(getPose().getRotation().getDegrees());
    }

}