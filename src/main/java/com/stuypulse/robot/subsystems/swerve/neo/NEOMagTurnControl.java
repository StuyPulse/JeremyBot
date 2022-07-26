package com.stuypulse.robot.subsystems.swerve.neo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.NEOModule.Turn;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.robot.util.NEOConfig;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NEOMagTurnControl extends TurnControl {
    private final CANSparkMax turn;
    private final DutyCycleEncoder encoder;

    private final SmartAngle offset; 

    public NEOMagTurnControl(int port, int encoderPort, SmartAngle radiansWhenForward) {
        super(Turn.Feedback.getController());

        turn = new CANSparkMax(port, MotorType.kBrushless);
        
        encoder = new DutyCycleEncoder(encoderPort);

        offset = radiansWhenForward;
        configure(Turn.getConfig());
    }

    public NEOMagTurnControl configure(NEOConfig config) {
        config.setup(turn);
        return this;
    }

    private double getRadians() {
        return SLMath.map(encoder.getAbsolutePosition(), 0.0, 1.0, -Math.PI, +Math.PI);
    }

    @Override
    public Rotation2d getAngle() {
        Rotation2d angle = new Rotation2d(getRadians());
        return angle.minus(offset.getRotation2d());
    }

    @Override
    protected void log(String id) {
        super.log(id);

        SmartDashboard.putNumber(id + "/Abs Position", Math.toDegrees(getRadians()));
    }

    @Override
    protected void setVoltage(double voltage) {
        turn.setVoltage(voltage);
    }

    @Override
    protected void reset() {
        DriverStation.reportWarning("Trying to reset absolute encoder, this is likely unintended behavior!", false);
    }
}
