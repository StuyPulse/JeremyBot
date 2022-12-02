package com.stuypulse.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Robot.Encoder;
import com.stuypulse.robot.subsystems.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WPI_SwerveModule extends SubsystemBase implements SwerveModule {
    
    /** CONSTANTS **/

    private interface Turn {
        double kP = 3.0;
        double kI = 0.0;
        double kD = 0.1;
    }

    private interface Drive {
        double kP = 1.6029;
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.11114;
        double kV = 2.7851;
        double kA = 0.30103;
    }

    /** MODULE **/
    
    private final String id;
    private final Translation2d moduleOffset;
    
    private SwerveModuleState targetState;
    private double lastTargetSpeed;
    
    /** TURNING **/
    
    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder absoluteEncoder;
    private final Rotation2d absoluteOffset;

    private final PIDController turnController;
    
    /** DRIVING */
    
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final PIDController driveController;
    private final SimpleMotorFeedforward driveFeedforward;

    public WPI_SwerveModule(String id, int turnId, int driveId, int encoderPort, Rotation2d absoluteOffset, Translation2d moduleOffset) {
        // Module
        this.id = id;
        this.moduleOffset = moduleOffset;

        targetState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        lastTargetSpeed = 0.0;

        // Turning
        turnMotor = new CANSparkMax(turnId, CANSparkMax.MotorType.kBrushless);
        Motors.TURN_CONFIG.config(turnMotor);
        
        absoluteEncoder = new DutyCycleEncoder(encoderPort);
        this.absoluteOffset = absoluteOffset;

        turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);
        turnController.enableContinuousInput(-Math.PI, +Math.PI);
       
        // Driving
        driveMotor = new CANSparkMax(driveId, CANSparkMax.MotorType.kBrushless);
        Motors.DRIVE_CONFIG.config(driveMotor);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        driveFeedforward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        
        // Network
        addChild("Absolute Encoder", absoluteEncoder);
        addChild("Turn Controller", turnController);
        addChild("Drive Controller", driveController);
    }

    /** MODULE METHODS **/
    
    public String getId() {
        return id;
    }

    public Translation2d getLocation() {
        return moduleOffset;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }
    
    /** TURNING METHODS **/

    public Rotation2d getAngle() {
        double radians = MathUtil.interpolate(-Math.PI, +Math.PI, absoluteEncoder.getAbsolutePosition());
        return new Rotation2d(radians).minus(absoluteOffset);
    }

    /** DRIVING METHODS **/
    
    public double getSpeed() {
        return driveEncoder.getVelocity();
    }
    
    /** CONTROL LOOP **/
    @Override
    public void periodic() {

        // Control Loops
        turnMotor.setVoltage(turnController.calculate(getAngle().getRadians(), targetState.angle.getRadians()));

        driveMotor.setVoltage(
            driveFeedforward.calculate(lastTargetSpeed, targetState.speedMetersPerSecond, 0.02) + 
            driveController.calculate(getSpeed(), targetState.speedMetersPerSecond)
        );

        lastTargetSpeed = targetState.speedMetersPerSecond;

        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", MathUtil.inputModulus(getAngle().getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Speed", getSpeed());
        
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Speed", targetState.speedMetersPerSecond);
    }
}
