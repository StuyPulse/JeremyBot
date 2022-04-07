package com.stuypulse.robot.subsystems;


import static com.stuypulse.robot.Constants.SwerveModule.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.Constants.SwerveModule.Feedback;
import com.stuypulse.robot.Constants.SwerveModule.Feedforward;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule2 extends SubsystemBase {
    private final Vector2D location; 

    private Polar2D target;

    private DriveWheel drive;

    private CANSparkMax pivot;
    private RelativeEncoder pivotEncoder;
    private Controller angleController;

    private String id;

    public SwerveModule2(String id, Vector2D location, int drivePort, int pivotPort) {
        this.id = id;

        this.location = location;

        target = new Polar2D(0, Angle.fromDegrees(0));

        // create & configure drive motor / encoder
        CANSparkMax driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(SMART_LIMIT);
        
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(METERS_CONVERSION);
        driveEncoder.setPositionConversionFactor(METERS_CONVERSION / 60.0);

        // create drive wheel system
        drive = new DriveWheel(
            driveMotor, driveEncoder, 
            Feedforward.getFeedforward(),
            Feedback.getController()
        );

        // create pivot system
        pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);
        pivot.setSmartCurrentLimit(SMART_LIMIT);
        pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPositionConversionFactor(PIVOT_CONVERSION);

        angleController = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D);
    }

    public SwerveModule2(Vector2D location, int drivePort, int pivotPort) {
        this(null, location, drivePort, pivotPort);
    }

    public String getID() {
        return id;
    }


    public double setTarget(SwerveModuleState target) {
        return setTarget(new Polar2D(
            target.speedMetersPerSecond, 
            Angle.fromDegrees(target.angle.getDegrees())
        ));
    }

    public void normalizeTarget(double magnitude) {
        target = target.div(magnitude);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drive.getVelocity(),
            getAngle().getRotation2d()
        );
    }

    public void reset() {
        drive.reset();

        if(isFlipped()) {
            pivotEncoder.setPosition(0);
        } else {
            pivotEncoder.setPosition(Math.PI);
        }
    }

    public double setTarget(Vector2D translation, double angular) {
        Vector2D perp = location.rotate(Angle.fromDegrees(90));
        Vector2D output = translation.add(perp.mul(angular));

        return setTarget(output.getPolar());
    }

    public double setTarget(Polar2D target) {
        this.target = new Polar2D(SLMath.deadband(target.magnitude, TARGET_DEADBAND), target.angle);
        return target.magnitude;
    }

    public Vector2D getLocation() {
        return location;
    }

    private Angle getTargetAngle() {
        return target.angle;
    }

    private Angle getRawAngle() {
        return Angle.fromRadians(pivotEncoder.getPosition());
    }

    private Angle getRawAngleError() {
        return getTargetAngle().sub(getRawAngle());
    }

    private boolean isFlipped() {
        return Math.abs(getRawAngleError().toDegrees())>90;
    }

    public Angle getAngle() {
        Angle angle = getRawAngle();
        return isFlipped() ? 
            angle.add(Angle.fromDegrees(180)) : 
            angle;
    }

    private Angle getAngleError() {
        return getTargetAngle().sub(getAngle());
    }

    private void setVelocity(double velocity) {
        if (isFlipped()) 
            velocity = -velocity;

        drive.setVelocity(velocity);
    }

    private void setAngularSpeed(double speed) {
        pivot.set(speed);
    }

    @Override
    public void periodic() {
        if(target.magnitude > MIN_ALIGN_MAGNITUDE) {
            Angle error = getAngleError();
            setVelocity(error.cos() * target.magnitude);
            
            double angularSpeed = angleController.update(error.toRadians());
            setAngularSpeed(angularSpeed);
        } else {
            drive.setVelocity(0);
            pivot.set(0);
        }

        if (id != null) {
            SmartDashboard.putNumber(id + "/Target Ang", getTargetAngle().toDegrees());
            SmartDashboard.putNumber(id + "/Encoder Ang", getAngle().toDegrees());

            SmartDashboard.putNumber(id + "/Target Speed", target.magnitude);
            SmartDashboard.putNumber(id + "/Error", getAngle().toDegrees());
        }
    }
}