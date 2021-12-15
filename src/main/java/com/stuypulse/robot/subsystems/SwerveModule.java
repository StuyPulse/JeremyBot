package com.stuypulse.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.stuypulse.robot.Constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.stuypulse.robot.Constants.SwerveModule.*;

// positive angular speed is ccw drive motor rotation
// it is important that the angle class's positive is ccw 
public class SwerveModule extends SubsystemBase {
    private Vector2D location;
    
    //
    private CANSparkMax drive; 
    private CANSparkMax pivot;

    private CANEncoder driveEncoder;
    private CANEncoder pivotEncoder;

    private Polar2D target;

    private Controller angleController;

    public SwerveModule(Vector2D location, int drivePort, int pivotPort) {
        this.location = location;
        target = new Polar2D(0, Angle.fromDegrees(0));

        drive = new CANSparkMax(drivePort, MotorType.kBrushless);
        pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

        driveEncoder = drive.getEncoder();
        pivotEncoder = pivot.getEncoder();

        angleController = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D);
    }

    public double setTarget(Vector2D translation, double angular) {
        Vector2D perp = location.rotate(Angle.fromDegrees(90));
        Vector2D output = translation.add(perp.mul(angular));
        return setTarget(output.getPolar());
    }

    public double setTarget(Polar2D target) {
        this.target = target;
        return target.magnitude;
    }

    public Vector2D getLocation() {
        return location;
    }

    public void normalizeLocation(double magnitude) {
        location = location.div(magnitude);
    }

    public void normalizeTarget(double magnitude) {
        target = target.div(magnitude);
    }

    private double getPivotRevolutions() {
        return pivotEncoder.getPosition();
    }

    private double getDriveRadians() {
        return getPivotRevolutions() * Constants.SwerveModule.REV_TO_RAD;
    }

    public Angle getAngle() {
        return Angle.fromRadians(getDriveRadians());
    }

    private Angle getTargetAngle() {
        return target.angle;
    }

    public boolean isFlipped() {
        return Math.abs(getRawAngleError().toDegrees())>90;
    }

    private Angle getRawAngleError() {
        // return getAngle().sub(getTargetAngle());
        return getTargetAngle().sub(getAngle());
    }

    private Angle getAngleError() {
        Angle error = getRawAngleError();
        return isFlipped() ? 
            error.add(Angle.fromDegrees(180)) : 
            error;
    }

    public void setSpeed(double speed) {
        if (isFlipped()) 
            speed = -speed;
        drive.set(speed);
    }

    public void setAngularSpeed(double speed) {
        pivot.set(speed);
    }

    @Override
    public void periodic() {
        Angle error = getAngleError();
        setSpeed(error.cos() * target.magnitude);
        
        double angularSpeed = angleController.update(error.toRadians());
        setAngularSpeed(angularSpeed);
    }
}