package com.stuypulse.robot.subsystems.modules.physical;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.NEOModule.Drive;
import com.stuypulse.robot.subsystems.modules.DriveControl;
import com.stuypulse.robot.util.NEOConfig;

public class NEODrive implements DriveControl.PhysicalControl {
    private final CANSparkMax drive;
    private final RelativeEncoder encoder;

    public NEODrive(int port) {
        drive = new CANSparkMax(port, MotorType.kBrushless);
        encoder = drive.getEncoder();

        configure(Drive.getConfig());
    }

    public NEODrive configure(NEOConfig config) {
        config.setup(drive);
        return this;
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setVoltage(double voltage) {
        drive.setVoltage(voltage);
    }

    @Override
    public void reset() {
        encoder.setPosition(0);
    }
}
