package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WPI_NEOModule extends SubsystemBase {
    
    /** CONSTANTS **/

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

    private interface Encoder {
        public interface Drive {
            double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
            
            public interface Stages {
                // input / output 
                double FIRST = 16.0 / 48.0;
                double SECOND = 28.0 / 16.0;
                double THIRD = 15.0 / 60.0;
            }

            double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }

        public interface Turn {
            double GEAR_RATIO = 1.0 / 12.8;
            double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI; 
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }

    private interface Motors {
        SparkMaxConfig DRIVE = new SparkMaxConfig(false, IdleMode.kCoast, 60, 0.0);
        SparkMaxConfig TURN = new SparkMaxConfig(false, IdleMode.kBrake, 60, 0.0);
    }

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

    public WPI_NEOModule(String id, int driveId, int turnId, int encoderPort, Rotation2d absoluteOffset, Translation2d location) {
        // Module
        this.id = id;
        this.location = location;

        targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        lastTargetSpeed = 0.0;

        // Driving
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        drivePID = driveMotor.getPIDController();
        driveFF = new SimpleMotorFeedforward(Drive.kS , Drive.kV, Drive.kA);

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

        drivePID.setP(Drive.kP);
        drivePID.setI(Drive.kI);
        drivePID.setD(Drive.kD);
        drivePID.setDFilter(1.0);

        Motors.DRIVE.config(driveMotor); // Do this last because burnFlash
        
        // Configure turn motors, sensors

        turnEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        seedTurnEncoder();

        turnPID.setP(Turn.kP);
        turnPID.setI(Turn.kI);
        turnPID.setD(Turn.kD);
        turnPID.setDFilter(1.0);

        Motors.TURN.config(turnMotor);
    }


    /** MODULE METHODS **/

    public String getId() {
        return id;
    }

    public Translation2d getModuleOffset() {
        return location;
    }

    /** DRIVE METHODS **/

    /** reads velocity from the built-in encoder */
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    /** TURN METHODS **/

    /** reads the non-zeroed absolute encoder value (forward may not be zero if this is called) */
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
        
        // read directly from encoder here so that when error is calculated on the SparkMax it is
        // cancelled out, and the leftover error is the shortest path (handled by using Rotation2d's)
        // to calculate error
        turnPID.setReference(getRadians() + error.getRadians(), ControlType.kPosition); 

        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", MathUtil.inputModulus(getRotation().getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Speed", getSpeed());
        
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Speed", targetState.speedMetersPerSecond);
    }
}
