package com.stuypulse.robot.subsystems.swerve.neo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.MagEncoder;
import com.stuypulse.robot.constants.NEOModule.Turn;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.robot.util.NEOConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NEOMagTurnControl extends TurnControl {
    private final CANSparkMax turn;
    private final DutyCycleEncoder encoder;

    public NEOMagTurnControl(int port, int encoderPort) {
        super(Turn.Feedback.getController().enableContinuous(MagEncoder.MIN_VALUE, MagEncoder.MAX_VALUE));

        turn = new CANSparkMax(port, MotorType.kBrushless);
        
        encoder = new DutyCycleEncoder(encoderPort);

        
        configure(Turn.getConfig());
    }

    public NEOMagTurnControl configure(NEOConfig config) {
        config.setup(turn);
        return this;
    }

    private double getRadians() {
        return encoder.getAbsolutePosition() * MagEncoder.CONVERSION - Math.PI;
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(getRadians());
    }

    @Override
    protected void setVoltage(double voltage) {
        turn.setVoltage(voltage);
    }

    @Override
    protected void reset() {
        DriverStation.reportWarning("Trying to reset absolute encoder, don't know what this means?", false);
        // encoder.reset();
        // encoder.setPosition(0);
    }
}
