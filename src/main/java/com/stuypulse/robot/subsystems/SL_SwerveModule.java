package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.util.SparkMaxConfig;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheSwerveModule extends SubsystemBase implements SwerveModule {

    public interface Turn {
        SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.0);
        SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0);
        SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.1);
    }

    private interface Drive {
        SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.16);
        SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0);
        SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0);

        SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.11114);
        SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.7851);
        SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.30103);
    }

    public interface Encoder {
        public interface Stages {
            // input / output
            double FIRST = 16.0 / 48.0;
            double SECOND = 28.0 / 16.0;
            double THIRD = 15.0 / 60.0;
        }

        double WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(4.0);
        double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

        double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
    }

    public interface Motors {
        SparkMaxConfig Turn = new SparkMaxConfig(false, IdleMode.kBrake, 40, 0.0);
        SparkMaxConfig Drive = new SparkMaxConfig(false, IdleMode.kBrake, 60, 0.0);
    }

    // module data

    private final String id;
    private final Translation2d location;

    private SwerveModuleState targetState;

    // turn

    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final SmartAngle angleOffset;

    private final AngleController turnController;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final Controller driveController;

    public TheSwerveModule(String id, Translation2d location, int turnCANId, int absoluteEncoderChannel,
            SmartAngle angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState();

        // turn

        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        Motors.Turn.config(turnMotor);

        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderChannel);
        this.angleOffset = angleOffset;
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        // drive

        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        Motors.Drive.config(driveMotor);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.VELOCITY_CONVERSION);

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .add(new Feedforward.Motor(Drive.kS, Drive.kV, Drive.kA).velocity());
    }

    @Override
    public String getId() {
        // TODO Auto-generated method stub
        return id;
    }

    @Override
    public Translation2d getModuleOffset() {
        // TODO Auto-generated method stub
        return location;
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        targetState = SwerveModuleState.optimize(state, getRotation2d());
    }

    private double getSpeed() {
        return driveEncoder.getVelocity();
    }

    private Rotation2d getAbsolutePosition() {
        return new Rotation2d(MathUtil.interpolate(-Math.PI, +Math.PI, absoluteEncoder.getAbsolutePosition()));
    }

    private Rotation2d getRotation2d() {
        return getAbsolutePosition().minus(angleOffset.getRotation2d());
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        return new SwerveModuleState(getSpeed(), getRotation2d());
    }

    @Override
    public void periodic() {
        turnMotor.setVoltage(turnController.update(
                Angle.fromRotation2d(targetState.angle),
                Angle.fromRotation2d(getRotation2d())));

        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber(id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber(id + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber(id + "/Absolute Angle", getAbsolutePosition().getDegrees());

        driveMotor.setVoltage(driveController.update(
                targetState.speedMetersPerSecond, getSpeed()));

        SmartDashboard.putNumber(id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Speed", getSpeed());
        SmartDashboard.putNumber(id + "/Speed Error", driveController.getError());
        SmartDashboard.putNumber(id + "/Speed Voltage", driveController.getOutput());

    }

}
