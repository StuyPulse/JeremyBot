package com.stuypulse.robot.subsystems.swerve.sim;

import com.stuypulse.robot.constants.SimModule.Drive;
import com.stuypulse.robot.subsystems.swerve.DriveControl;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimDriveControl extends DriveControl {
    private LinearSystemSim<N1, N1, N1> velocity;
    private StopWatch timer;

    private double voltage;

    public SimDriveControl() {
        super(Drive.Feedforward.getFeedforward(), Drive.Feedback.getController());
        velocity = new LinearSystemSim<>(Drive.Feedforward.getPlant());
        voltage = 0.0;

        timer = new StopWatch();
    }

    @Override
    public double getVelocity() {
        return velocity.getOutput(0);
    }

    @Override
    protected void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    protected void reset() {
    }

    protected void log(String id) {
        super.log(id);

        SmartDashboard.putNumber(id + "/Drive Voltage", voltage);
    }
        
    @Override
    public void periodic() {
        super.periodic();
        velocity.setInput(voltage);
        velocity.update(timer.reset());
    }
}
