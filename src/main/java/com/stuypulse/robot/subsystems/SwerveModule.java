package com.stuypulse.robot.subsystems;


import static com.stuypulse.robot.Constants.SwerveModule.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private final Vector2D location; 
    private Vector2D normalLocation;

    private CANSparkMax drive;
    private CANSparkMax pivot;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder pivotEncoder;

    private Polar2D target;

    private Controller angleController;

    private String id;

    public SwerveModule(String id, Vector2D location, int drivePort, int pivotPort) {
        this.id = id;

        this.location = location;
        this.normalLocation = location.normalize();

        target = new Polar2D(0, Angle.fromDegrees(0));

        drive = new CANSparkMax(drivePort, MotorType.kBrushless);
        pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

        drive.setSmartCurrentLimit(SMART_LIMIT);
        pivot.setSmartCurrentLimit(SMART_LIMIT);

        driveEncoder = drive.getEncoder();
        pivotEncoder = pivot.getEncoder();

        driveEncoder.setPositionConversionFactor(METERS_CONVERSION);
        driveEncoder.setVelocityConversionFactor(METERS_CONVERSION / 60.0);
        pivotEncoder.setPositionConversionFactor(PIVOT_CONVERSION);

        angleController = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D);
    }

    public SwerveModule(Vector2D location, int drivePort, int pivotPort) {
        this(null, location, drivePort, pivotPort);
    }

    public String getID() {
        return id;
    }

    public void reset() {
        driveEncoder.setPosition(0);

        if(isFlipped()) {
            pivotEncoder.setPosition(0);
        } else {
            pivotEncoder.setPosition(Math.PI);
        }
    }

    public double setTarget(Vector2D translation, double angular) {
        Vector2D perp = normalLocation.rotate(Angle.fromDegrees(90));
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

    public void normalizeLocation(double maxMagnitude) {
        normalLocation = location.div(maxMagnitude);
    }

    public void normalizeTarget(double magnitude) {
        target = target.div(magnitude);
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

    private void setSpeed(double speed) {
        if (isFlipped()) 
            speed = -speed;
        drive.set(speed);
    }

    private void setAngularSpeed(double speed) {
        pivot.set(speed);
    }

    @Override
    public void periodic() {
        if(target.magnitude > MIN_ALIGN_MAGNITUDE) {
            Angle error = getAngleError();
            setSpeed(error.cos() * target.magnitude);
            
            double angularSpeed = angleController.update(error.toRadians());
            setAngularSpeed(angularSpeed);
        } else {
            drive.set(0);
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