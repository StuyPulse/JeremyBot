package com.stuypulse.robot.subsystems.swerve.neo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.robot.util.NEOConfig;
import com.stuypulse.stuylib.control.Controller;

import edu.wpi.first.math.geometry.Rotation2d;

public class NEOTurnControl extends TurnControl {
    private final CANSparkMax turn;
    private final RelativeEncoder encoder;

    public NEOTurnControl(int port, Controller feedback) {
        super(feedback);

        turn = new CANSparkMax(port, MotorType.kBrushless);
        encoder = turn.getEncoder();
    }

    public NEOTurnControl configure(NEOConfig config) {
        config.setup(turn);
        return this;
    }

    private double getRadians() {
        return encoder.getPosition();
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(getRadians());
    }

    @Override
    protected void setVoltage(double voltage) {
        turn.setVoltage(voltage);
    }
}
