package com.stuypulse.robot.subsystems.modules.physical;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.MagEncoder;
import com.stuypulse.robot.constants.NEOModule.Turn;
import com.stuypulse.robot.subsystems.modules.TurnControl;
import com.stuypulse.robot.util.NEOConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NEOMagTurn implements TurnControl.PhysicalControl {
    private final CANSparkMax turn;
    private final DutyCycleEncoder encoder;

    // what the encoder reads when the drive is facing forward,
    // meaning we have to subtract this from the absolute encoder
    // reading to get back to forward being 0 radians
    private final Rotation2d offset; 

    public NEOMagTurn(int port, int encoderPort, Rotation2d readingWhenForward) {
        turn = new CANSparkMax(port, MotorType.kBrushless);
        
        encoder = new DutyCycleEncoder(encoderPort);

        offset = readingWhenForward;
        configure(Turn.getConfig());
    }

    public NEOMagTurn(int port, int encoderPort) {
        this(port, encoderPort, new Rotation2d());
    }

    public NEOMagTurn configure(NEOConfig config) {
        config.setup(turn);
        return this;
    }

    private double getRadians() {
        return MagEncoder.getRadians(encoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(getRadians()).minus(offset);
    }

    @Override
    public void setVoltage(double voltage) {
        turn.setVoltage(voltage);
    }

    @Override
    public void reset() {
        DriverStation.reportWarning("Trying to reset absolute encoder, this is likely unintended behavior!", false);
    }
}
