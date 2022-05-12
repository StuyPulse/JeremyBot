package com.stuypulse.robot.subsystems.swerve.neo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.NEOModule.Drive;
import com.stuypulse.robot.subsystems.swerve.DriveControl;
import com.stuypulse.robot.util.NEOConfig;

public class NEODriveControl extends DriveControl {
    private final CANSparkMax drive;
    private final RelativeEncoder encoder;

    public NEODriveControl(int port) {
        super(Drive.Feedforward.getFeedforward(), Drive.Feedback.getController());

        drive = new CANSparkMax(port, MotorType.kBrushless);
        encoder = drive.getEncoder();

        configure(Drive.getConfig());
    }

    public NEODriveControl configure(NEOConfig config) {
        config.setup(drive);
        return this;
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    protected void setVoltage(double voltage) {
        drive.setVoltage(voltage);
    }
}
