package com.stuypulse.robot.subsystems.swerve.sim;

import com.stuypulse.robot.constants.SimModule;
import com.stuypulse.robot.constants.SimModule.Drive;
import com.stuypulse.robot.subsystems.swerve.DriveControl;
import com.stuypulse.robot.util.MotorSim;

public class SimDriveControl extends DriveControl {
    private final MotorSim sim;

    public SimDriveControl() {
        super(Drive.Feedforward.getFeedforward(), Drive.Feedback.getController());

        sim = new MotorSim(Drive.Feedforward.getPlant(), Drive.Encoder.GEARING, SimModule.WHEEL_DIAMETER);
    }

    @Override
    public double getVelocity() {
        return sim.getSpeedMetersPerSecond();
    }

    @Override
    protected void setVoltage(double voltage) {
        sim.setVoltage(voltage);
    }

    @Override
    protected void reset() {
        sim.reset();
    }

    @Override
    public void simulationPeriodic() {
        sim.update(0.020);
    }
}
