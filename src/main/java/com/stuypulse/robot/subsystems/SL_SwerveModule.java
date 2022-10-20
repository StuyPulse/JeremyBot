package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SL_SwerveModule extends SubsystemBase implements SwerveModule {
    private interface Turn {
        double kP = 3.0;
        double kI = 0.0;
        double kD = 0.1;
    }

    private interface Drive {
        double kP = 0.16;
        double kI = 0.00;
        double kD = 0.00;

        double kS = 0.11114;
        double kV = 2.7851;
        double kA = 0.30103;
    }

    /** HARDWARE */
    private final String id;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder turnEncoder;

    /** CONTROLLERS */

    private final Controller driveController;
    private final AngleController turnController;

    private final SwerveModuleState target;

    private final Translation2d location;
    private final Rotation2d turnOffset;

    public SL_SwerveModule(String id, int turnId, int driveId, int encoderId, Rotation2d absoluteOffset,
            Translation2d location) {
        setSubsystem("SwerveModule[" + id + "]");
        // Module
        this.id = id;
        this.location = location;

        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);

        turnEncoder = new DutyCycleEncoder(encoderId);
        driveEncoder = driveMotor.getEncoder();

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .add(new Feedforward.Motor(Drive.kS, Drive.kV, Drive.kA).velocity());
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        target = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        turnOffset = absoluteOffset;
    }

    @Override
    public String getId() {
        return id;
    }

    @Override
    public Translation2d getModuleOffset() {
        return location;
    }

    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return new Rotation2d(turnEncoder.get());
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, turnOffset);
        target.angle = state.angle;
        target.speedMetersPerSecond = state.speedMetersPerSecond;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    @Override
    public void periodic() {
        double turnOutput = turnController.update(Angle.fromRadians(turnEncoder.getAbsolutePosition()),
                Angle.fromDegrees(SwerveModuleState.optimize(target, turnOffset).angle.getDegrees()));
        turnMotor.setVoltage(turnOutput);
        double driveOutput = driveController.update(driveEncoder.getVelocity(), target.speedMetersPerSecond);
        driveMotor.setVoltage(driveOutput);

        SmartDashboard.putNumber("Turn Voltage", turnOutput);
        SmartDashboard.putNumber("Drive Voltage", driveOutput);
    }
}
