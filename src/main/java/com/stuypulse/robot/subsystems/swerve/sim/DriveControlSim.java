package com.stuypulse.robot.subsystems.swerve.sim;

import com.stuypulse.robot.constants.SimModule.Drive;
import com.stuypulse.robot.subsystems.swerve.DriveControl;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class DriveControlSim extends DriveControl {
    private final FlywheelSim drive;
    private final Encoder encoder;
    private final EncoderSim encoderSim;

    public DriveControlSim() {
        super(Drive.Feedforward.getFeedforward(), Drive.Feedback.getController());

        drive = new FlywheelSim(
            Drive.Feedforward.getPlant(),
            DCMotor.getNEO(1),
            Drive.Encoder.GEARING
        );

        encoder = new Encoder(Drive.Encoder.PORT_A, Drive.Encoder.PORT_B);
        encoder.setDistancePerPulse(Drive.Encoder.DISTANCE_PER_PULSE);

        encoderSim = new EncoderSim(encoder);
    }

    @Override
    public double getVelocity() {
        return drive.getAngularVelocityRPM();
    }

    @Override
    protected void setVoltage(double voltage) {
        drive.setInputVoltage(voltage);
    }

    @Override
    protected void reset() {
        encoderSim.setCount(0);
    }
}
