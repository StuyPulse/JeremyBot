package com.stuypulse.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot.Encoder;
import com.stuypulse.robot.constants.Settings.Robot.Control.Drive;
import com.stuypulse.robot.constants.Settings.Robot.Control.Turn;
import com.stuypulse.robot.subsystems.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WPI_NEOModule extends SubsystemBase implements SwerveModule {

    /** MODULE **/

    private final String id;
    private final Translation2d location;

    private SwerveModuleState targetState;
    private double lastTargetSpeed;

    /** DRIVING **/

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final SparkMaxPIDController drivePID;
    private final SimpleMotorFeedforward driveFF;

    /** TURNING **/

    private final CANSparkMax turnMotor;
    private final RelativeEncoder turnEncoder;
    private final DutyCycleEncoder turnAbsoluteEncoder;
    private final Rotation2d turnAbsoluteOffset;

    private final SparkMaxPIDController turnPID;

    public WPI_NEOModule(String id, int driveId, int turnId, int encoderPort, Rotation2d absoluteOffset,
            Translation2d location) {

        setSubsystem("SwerveModule[" + id + "]");
        // Module
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        lastTargetSpeed = 0.0;

        // Driving
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        drivePID = driveMotor.getPIDController();
        driveFF = new SimpleMotorFeedforward(Drive.kS.get(), Drive.kV.get(), Drive.kA.get());

        // Turning
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();

        turnAbsoluteEncoder = new DutyCycleEncoder(encoderPort);
        turnAbsoluteOffset = absoluteOffset;

        turnPID = turnMotor.getPIDController();

        // Configuration
        configure();

        // Logging
        addChild("Absolute Encoder", turnAbsoluteEncoder);
        // addChild("Turn Motor", turnMotor);
        // addChild("Drive Motor", driveMotor);
    }

    /** CONFIGURATION **/

    private void seedTurnEncoder() {
        turnEncoder.setPosition(getAbsoluteRadians());
    }

    private void configure() {
        // Configure drive motors, sensors

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveEncoder.setPosition(0.0);

        drivePID.setP(Drive.kP.get());
        drivePID.setI(Drive.kI.get());
        drivePID.setD(Drive.kD.get());
        drivePID.setDFilter(1.0);

        Motors.DRIVE_CONFIG.config(driveMotor); // Do this last because burnFlash

        // Configure turn motors, sensors

        turnEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        seedTurnEncoder();

        turnPID.setP(Turn.kP.get());
        turnPID.setI(Turn.kI.get());
        turnPID.setD(Turn.kD.get());
        turnPID.setDFilter(1.0);

        Motors.TURN_CONFIG.config(turnMotor);
    }

    /** MODULE METHODS **/

    public String getId() {
        return id;
    }

    public Translation2d getLocation() {
        return location;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getRotation());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getRotation());
    }

    /** DRIVE METHODS **/

    /** reads velocity from the built-in encoder */
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    /** TURN METHODS **/

    /**
     * reads the non-zeroed absolute encoder value (forward may not be zero if this
     * is called)
     */
    private double getRawAbsoluteRadians() {
        return MathUtil.interpolate(-Math.PI, +Math.PI, turnAbsoluteEncoder.getAbsolutePosition());
    }

    /** reads radians directly from the absolute encoder */
    public double getAbsoluteRadians() {
        return getRawAbsoluteRadians() - turnAbsoluteOffset.getRadians();
    }

    /**
     * stores the angle from the absolute encoder in a Rotation2d
     * so that operations on angles may have their value normalized
     * in the range (-Math.PI, +Math.PI)
     */
    public Rotation2d getAbsoluteRotation() {
        return new Rotation2d(getAbsoluteRadians());
    }

    /** reads radians directly from the built-in encoder */
    public double getRadians() {
        return turnEncoder.getPosition();
    }

    /**
     * stores the angle from the build-in encoder in a Rotation2d so
     * that operations on angles may have their value normalized in the
     * range (-Math.PI, +Math.PI)
     */
    public Rotation2d getRotation() {
        return new Rotation2d(getRadians());
    }

    @Override
    public void periodic() {
        // Update drive control loop
        double ff = driveFF.calculate(lastTargetSpeed, targetState.speedMetersPerSecond, Settings.dT);
        lastTargetSpeed = targetState.speedMetersPerSecond;

        drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity, 0, ff, ArbFFUnits.kVoltage);

        // Update angle control loop
        Rotation2d error = targetState.angle.minus(getRotation());

        // read directly from encoder here so that when error is calculated on the
        // SparkMax it is
        // cancelled out, and the leftover error is the shortest path (handled by using
        // Rotation2d's)
        // to calculate error
        turnPID.setReference(getRadians() + error.getRadians(), ControlType.kPosition);

        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Angle",
                MathUtil.inputModulus(getRotation().getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Speed", getSpeed());

        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle",
                MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Speed", targetState.speedMetersPerSecond);

        SmartDashboard.putNumber("Swerve/" + id + "/Absolute Angle",
                Math.toDegrees(getRawAbsoluteRadians()));
    }

}
