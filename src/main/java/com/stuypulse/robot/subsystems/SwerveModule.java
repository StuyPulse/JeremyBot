package com.stuypulse.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.stuypulse.robot.Constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Polar2D;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private final Vector2D location;
    
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

        // TODO: constants
        angleController = new PIDController(-1,-1,-1);
    }

    public double setTarget(Vector2D translation, double angular) {
        Vector2D perp = location.rotate(Angle.fromDegrees(90));
        
        // TODO: perpendicular is in m/s and translation is in 
        // x-box units (meaningless unit between [-1,1] / in the unit circle)
        // For now: we will make the perpendicular in x-box units as well, by 
        // normalizing, thereby putting it on the unit circle
        perp = perp.normalize();
        
        Vector2D output = translation.add(perp.mul(angular));

        return setTarget(output.getPolar());
    }

    public double setTarget(Polar2D target) {
        this.target = target;
        return target.magnitude;
    }

    public void normalize(double magnitude) {
        target = target.div(magnitude);
    }

    // public double getRawAngle() {
    //     return pivotEncoder.getPosition();
    // }

    public double getPivotRevolutions() {
        return pivotEncoder.getPosition();
    }

    public double getDriveRadians() {
        return getPivotRevolutions() * Constants.SwerveModule.REV_TO_RAD;
    }

    public Angle getAngle() {
        return Angle.fromRadians(getDriveRadians());
    }

    public Angle getTargetAngle() {
        return target.angle;
    }

    public Angle getAngleError() {
        return getAngle().sub(getTargetAngle());
    }

    @Override
    public void periodic() {
        Angle error = getAngleError();
        drive.set(SLMath.fpow(error.cos(), 3) * target.magnitude);
        
    }
}