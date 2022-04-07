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

/**
 * A class to control an individual swerve module in any 
 * swerve drive system.
 * 
 * @author Myles Pasetsky (@selym3)
 * @author Sam Belliveau (sam.Belliveau@gmail.com)
 */
public class SwerveModule extends SubsystemBase {
    private String id;
    
    /** Module data */
    private final Vector2D location;
    
    /** Target State */
    private Polar2D target;

    /** Drive motor control */
    private DriveWheel drive;

    /** Pivot motor control */
    private CANSparkMax pivot;
    private RelativeEncoder pivotEncoder;
    private Controller angleController;

    public SwerveModule(String id, Vector2D location, int drivePort, int pivotPort) {
        this.id = id;

        this.location = location;

        target = new Polar2D(0, Angle.fromDegrees(0));

        /** Create Drive system  */
        CANSparkMax driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(SMART_LIMIT);
        
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(METERS_CONVERSION);
        driveEncoder.setPositionConversionFactor(METERS_CONVERSION / 60.0);

        drive = new DriveWheel(driveMotor, driveEncoder, Feedforward.getFeedforward(), Feedback.getController());

        /** Create Pivot system */
        pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);
        pivot.setSmartCurrentLimit(SMART_LIMIT);
        
        pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPositionConversionFactor(PIVOT_CONVERSION);

        angleController = new PIDController(ANGLE_P, ANGLE_I, ANGLE_D);
    }

    public String getID() {
        return id;
    }

    /********************
     * SET TARGET STATE *
     ********************/

    /** 
     * Directly set the target state using a StuyLib Polar2d.
     * 
     * These methods return the magnitude (velocity) of the state so that 
     * other modules being updated can all be normalized if needed (e.g. 
     * one module goes over a max allowed magnitude)
     */
    public double setTarget(Polar2D target) {
        this.target = new Polar2D(SLMath.deadband(target.magnitude, TARGET_DEADBAND), target.angle);
        return target.magnitude;
    }

    /** Set the target state using WPILib SwerveModuleState */
    public double setTarget(SwerveModuleState target) {
        return setTarget(new Polar2D(
            target.speedMetersPerSecond, 
            Angle.fromDegrees(target.angle.getDegrees())
        ));
    }

    /** 
     * Sets the target based on linear and angular velocity, usually used
     * for driving from gamepad inputs.
     * 
     * In order to achieve a velocity and angular velocity, each module in a drive
     * system will not to be going a certain speed while at a particular angle (NOTE: not angular
     * velocity). 
     */
    public double setTarget(Vector2D velocity, double angular) {
        Vector2D perp = location.rotate(Angle.k90deg);
        Vector2D output = velocity.add(perp.mul(angular));

        return setTarget(output.getPolar());
    }

    /** 
     * Target may need to be normalized if another module in the same drive system
     * has a target magnitude that exceeds a maximum allowed magnitude. 
     * 
     * Clamping is not a good idea in this case because proportionality between locations should be
     * retained to rotate properly, so every module's target will be normalized to the maximum.
     */
    public void normalizeTarget(double maxMagnitude, double maxAllowed) {
        target = target.div(maxMagnitude).mul(maxAllowed);
    }

    /****************
     * ANGLE VALUES *
     ****************/
    
    /** Reads from whatever the target state is */
    private Angle getTargetAngle() {
        return target.angle;
    }

    /** Reads from the pivot encoder, which has a conversion factor to drive radians */
    private Angle getRawAngle() {
        return Angle.fromRadians(pivotEncoder.getPosition());
    }

    /** 
     * Gets angle error, unadjusted for any speed optimizations that arise when
     * the error is on a circle and there may be a smaller error that reaches the 
     * same setpoint (e.g. 179 degrees off vs 1 degree off)
     */
    private Angle getRawAngleError() {
        return getTargetAngle().sub(getRawAngle());
    }

    /** 
     * Returns whether or not to flip angle error and speed setpoint 
     * (to adjust for drive wheel not being turned around, what's forward
     * doesn't change) 
     */
    private boolean isFlipped() {
        return Math.abs(getRawAngleError().toDegrees()) > 90;
    }

    /** Returns angle of drive wheel, adjusted by 180 deg if needs to flipped */
    public Angle getAngle() {
        Angle angle = getRawAngle();
        return isFlipped() ? 
            angle.add(Angle.k180deg) : 
            angle;
    }

    /** Returns optimized angle error */
    private Angle getAngleError() {
        return getTargetAngle().sub(getAngle());
    }

    /*******************
     * VELOCITY VALUES *
     *******************/

    /** Sets the target velocity, adjusting for if the module is flipped */
    private void setVelocity(double velocity) {
        if (isFlipped()) 
            velocity = -velocity;

        drive.setVelocity(velocity);
    }



    /************
     * ODOMETRY *
     ************/

    /** 
     * Returns the actual state of the module, useful for odometry, which will
     * use the actual states of several modules.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drive.getVelocity(),
            getAngle().getRotation2d()
        );
    }

    /** Returns the location of the module, useful for calculuting desired states */
    public Vector2D getLocation() {
        return location;
    }

    /*****************
     * MISC. CONTROL *
     *****************/

    /** Resets positions of drive and pivot encoders */
    public void reset() {
        drive.reset();
        pivotEncoder.setPosition(isFlipped() ? 0 : Math.PI);
    }

    /** Stops drive and pivot motors */
    public void stop() {
        target = new Polar2D(0, getRawAngle());

        drive.setVelocity(0);
        pivot.set(0);
    }


    @Override
    public void periodic() {
        if(target.magnitude > MIN_ALIGN_MAGNITUDE) {
            Angle error = getAngleError();

            // cosine of angle error will mitigate velocity when
            // target is off its target angle, prevents fighting other modules
            setVelocity(error.cos() * target.magnitude); 
            
            // angle is controlled using PID
            double angularSpeed = angleController.update(error.toRadians());
            pivot.set(angularSpeed);
        } 
        
        else {
            stop();
        }

        SmartDashboard.putNumber(id + "/Target Ang", getTargetAngle().toDegrees());
        SmartDashboard.putNumber(id + "/Encoder Ang", getAngle().toDegrees());

        SmartDashboard.putNumber(id + "/Target Speed", target.magnitude);
        SmartDashboard.putNumber(id + "/Error", getAngle().toDegrees());
    }
}