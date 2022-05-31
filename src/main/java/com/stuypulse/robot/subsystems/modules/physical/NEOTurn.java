package com.stuypulse.robot.subsystems.modules.physical;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.NEOModule.Turn;
import com.stuypulse.robot.subsystems.modules.TurnControl;
import com.stuypulse.robot.util.NEOConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class NEOTurn implements TurnControl.PhysicalControl {
    private final CANSparkMax turn;
    private final RelativeEncoder encoder;

    public NEOTurn(int port) {
        turn = new CANSparkMax(port, MotorType.kBrushless);
        encoder = turn.getEncoder();

        configure(Turn.getConfig());
    }

    public NEOTurn configure(NEOConfig config) {
        config.setup(turn);
        return this;
    }

    private double getRadians() {
        return encoder.getPosition();
    }

    @Override
    public Rotation2d getAngle() {
        // readings are not continuous but this gets handled
        // by rotation2d plus/minus
        return new Rotation2d(getRadians());
    }

    @Override
    public void setVoltage(double voltage) {
        turn.setVoltage(voltage);
    }

    @Override
    public void reset() {
        encoder.setPosition(0);
    }
}
