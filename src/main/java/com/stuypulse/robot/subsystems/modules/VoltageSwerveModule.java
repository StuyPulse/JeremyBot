package com.stuypulse.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Robot.Encoder;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VoltageSwerveModule extends SubsystemBase {

    public interface Turn {
        SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
        SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
        SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.0);
    }

    // module data

    private final String id;
    private final Translation2d location;

    // turn

    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final SmartAngle angleOffset;

    private final AngleController turnController;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private double voltage;

    public VoltageSwerveModule(String id, Translation2d location, int turnCANId, int absoluteEncoderChannel,
            SmartAngle angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;

        // turn

        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        Motors.TURN_CONFIG.config(turnMotor);

        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderChannel);
        this.angleOffset = angleOffset;
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        // drive

        voltage = 0.0;

        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        Motors.DRIVE_CONFIG.config(driveMotor);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

    }

    public String getId() {
        return id;
    }

    public Translation2d getModuleOffset() {
        return location;
    }

    private double getVelocity() {
        return driveEncoder.getVelocity();
    }

    private Rotation2d getAbsolutePosition() {
        return new Rotation2d(MathUtil.interpolate(-Math.PI, +Math.PI, absoluteEncoder.getAbsolutePosition()));
    }

    private Rotation2d getRotation2d() {
        return getAbsolutePosition().minus(angleOffset.getRotation2d());
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void periodic() {
        turnMotor.setVoltage(turnController.update(Angle.kZero, Angle.fromRotation2d(getRotation2d())));
        driveMotor.setVoltage(voltage);

        SmartDashboard.putNumber(id + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber(id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber(id + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber(id + "/Absolute Angle", getAbsolutePosition().getDegrees());

        SmartDashboard.putNumber(id + "/Velocity", getVelocity());

        SmartDashboard.putNumber(id + "/Drive Voltage", voltage);
    }

}
