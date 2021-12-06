package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.*;

import com.revrobotics.CANSparkMax;

// import com.stuypulse.

public class SwerveModule extends Subsystem {
    private final Vector2D location;
    
    //
    private CANSparkMax drive; 
    private CANSparkMax pivot;

    // private CANEncoder driveEncoder;
    // private CANEncoder pivotEncoder;

    private Polar2D target;

    private PIDController driveController;

    public SwerveModule(Vector2D location, int drivePort, int pivotPort) {
        this.location = location;
        target = new Polar2D(0, Angle.fromDegrees(0));

        drive = new CANSparkMax(drivePort, MotorType.kBrushless);
        pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

        driveController = new PIDController(/**/);
    }

    public double setTarget(Vector2D translation, double angular) {
        return setTarget(
            translation.add()
        );
    }
    
    public double setTarget(Polar2D target) {
        this.target = target;
        return target.magnitude;
    }

    // public void getTarget()

    public Angle getAngle() {
        return null;
    }

    public double getMotorSpeed() {
        return drive.get();
    }

    public double getTargetMotorSpeed() {
        return target.magnitude;
    }

    public double getMotorSpeedError() {
        // return getMotorSpeed() -
    }

    @Override
    public void periodic() {
        double speedError = getMotorSpeed() - target.magnitude;
        double 
    }
}