package com.stuypulse.robot.subsystems.modules;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.robot.util.ADelayFilter;
import com.stuypulse.robot.util.DelayFilter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.*;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SL_SimModule extends SubsystemBase implements SwerveModule {
    
    /** CONSTANTS **/

    private interface Turn {
        double kP = 3.0; // 1.2 with ff
        double kI = 0.0;
        double kD = 0.1; // 0.04 with ff

        double kS = 0.14;
        double kV = 0.25;
        double kA = 0.007;
    }

    private interface Drive {
        double kP = 1.6;
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

    /** TURNING **/
    
    private final LinearSystemSim<N2, N1, N1> turnSim;
    private double turnVoltage;

    private final AngleController turnController;
    
    /** DRIVING */
    
    private final LinearSystemSim<N1, N1, N1> driveSim;
    private double driveVoltage;

    private final Controller driveController;

    public SL_SimModule(String id, Translation2d moduleOffset) {
        // Module
        this.id = id;
        this.moduleOffset = moduleOffset;

        targetState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

        // Turning
        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV, Turn.kA));
        turnVoltage = 0.0;
        
        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            // .add(new Feedforward.Motor(Turn.kS, Turn.kV, Turn.kA).angle())
        ;

        // Driving
        driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(Drive.kV, Drive.kA));
        driveVoltage = 0.0;

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .setSetpointFilter(new DelayFilter())
            .add(new Feedforward.Motor(Drive.kS, Drive.kV, Drive.kA).velocity());
        
    }

    /** MODULE METHODS **/
    
    public String getId() {
        return id;
    }

    public Translation2d getModuleOffset() {
        return moduleOffset;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle().getRotation2d());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle().getRotation2d());
    }
    
    /** TURNING METHODS **/

    public Angle getAngle() {
        return Angle.fromRadians(turnSim.getOutput(0));
    }

    /** DRIVING METHODS **/
    
    public double getSpeed() {
        return driveSim.getOutput(0);
    }
    
    /** CONTROL LOOP **/
    @Override
    public void periodic() {

        // Control Loops
        turnVoltage = turnController.update(Angle.fromRotation2d(targetState.angle), getAngle());
        driveVoltage = driveController.update(targetState.speedMetersPerSecond, getSpeed());

        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", MathUtil.inputModulus(getAngle().toDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Speed", getSpeed());
        
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Speed", targetState.speedMetersPerSecond);
    }

    @Override
    public void simulationPeriodic() {
        // Drive Simulation
        driveSim.setInput(driveVoltage);

        driveSim.update(Settings.dT);

        // Turn Simulation
        turnSim.setInput(turnVoltage);

        turnSim.update(Settings.dT);

        // Robot Simulation
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));
    }
}
