package com.stuypulse.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Robot.Encoder;
import com.stuypulse.robot.constants.Settings.Robot.Control.Drive;
import com.stuypulse.robot.constants.Settings.Robot.Control.Turn;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
// import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SL_SwerveModule extends SubsystemBase implements SwerveModule {


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

    public SL_SwerveModule(String id, Translation2d location, int turnCANId, int absoluteEncoderChannel,
            SmartAngle angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState();

        // turn

        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        Motors.TURN_CONFIG.config(turnMotor);

        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderChannel);
        this.angleOffset = angleOffset;
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        // drive

        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        // driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        Motors.DRIVE_CONFIG.config(driveMotor);
        

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .add(new Feedforward.Motor(Drive.kS, Drive.kV, Drive.kA).velocity());
    }

    @Override
    public String getId() {
        // TODO Auto-generated method stub
        return id;
    }

    @Override
    public Translation2d getLocation() {
        // TODO Auto-generated method stub
        return location;
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        targetState = SwerveModuleState.optimize(state, getRotation2d());
    }

    private double getSpeed() {
        return driveEncoder.getVelocity(); // * Encoder.Drive.VELOCITY_CONVERSION;
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

        driveMotor.setVoltage(Math.abs(turnController.getError().cos()) * driveController.update(
                targetState.speedMetersPerSecond, getSpeed()));

        SmartDashboard.putNumber(id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Speed", getSpeed());
        SmartDashboard.putNumber(id + "/Speed Error", driveController.getError());
        SmartDashboard.putNumber(id + "/Speed Voltage", driveController.getOutput());

    }

}