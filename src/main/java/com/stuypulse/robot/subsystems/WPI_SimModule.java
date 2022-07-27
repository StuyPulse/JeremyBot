package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

public class WPI_SimModule extends SubsystemBase {
    
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
    
    private final LinearSystemSim<N2, N1, N1> turnSim;
    private double turnVoltage;

    private final PIDController turnController;
    
    /** DRIVING */
    
    private final LinearSystemSim<N1, N1, N1> driveSim;
    private double driveVoltage;

    private final PIDController driveController;
    private final SimpleMotorFeedforward driveFeedforward;

    public WPI_SimModule(String id, int turnId, int driveId, int encoderPort, Rotation2d absoluteOffset, Translation2d moduleOffset) {
        // Module
        this.id = id;
        this.moduleOffset = moduleOffset;

        targetState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        lastTargetSpeed = 0.0;

        // Turning
        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV, Turn.kA));
        turnVoltage = 0.0;
        
        turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);
        turnController.enableContinuousInput(-Math.PI, +Math.PI);
       
        // Driving
        driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(Drive.kV, Drive.kA));
        driveVoltage = 0.0;

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        driveFeedforward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        
        // Network
        addChild("Turn Controller", turnController);
        addChild("Drive Controller", driveController);
    }

    /** MODULE METHODS **/
    
    public String getId() {
        return id;
    }

    public Translation2d getModuleOffset() {
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
        return new Rotation2d(turnSim.getOutput(0));
    }

    /** DRIVING METHODS **/
    
    public double getSpeed() {
        return driveSim.getOutput(0);
    }
    
    /** CONTROL LOOP **/
    @Override
    public void periodic() {

        // Control Loops
        turnVoltage = turnController.calculate(getAngle().getRadians(), targetState.angle.getRadians());

        driveVoltage = 
            driveFeedforward.calculate(lastTargetSpeed, targetState.speedMetersPerSecond, 0.02) + 
            driveController.calculate(getSpeed(), targetState.speedMetersPerSecond);

        lastTargetSpeed = targetState.speedMetersPerSecond;

        // Network Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", MathUtil.inputModulus(getAngle().getDegrees(), -180, +180));
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
