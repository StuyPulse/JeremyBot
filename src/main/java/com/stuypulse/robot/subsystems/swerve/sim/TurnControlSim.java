package com.stuypulse.robot.subsystems.swerve.sim;

import com.stuypulse.robot.constants.SimModule;
import com.stuypulse.robot.constants.SimModule.Turn;
import com.stuypulse.robot.subsystems.swerve.TurnControl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TurnControlSim extends TurnControl {
	private final FlywheelSim turn;
	private final Encoder encoder;
	private final EncoderSim encoderSim;

	public TurnControlSim() {
		super(Turn.Feedback.getController());

		turn = new FlywheelSim(
			Turn.Feedforward.getPlant(),
			DCMotor.getNEO(1),
			Turn.Encoder.GEAR_RATIO
		);

		encoder = new Encoder(Turn.Encoder.PORT_A, Turn.Encoder.PORT_B);
		encoder.setDistancePerPulse(Turn.Encoder.GRAYHILL_DISTANCE_PER_PULSE);

		encoderSim = new EncoderSim(encoder);
	}

	private double getRadians() {
		double rotations = encoder.getDistance() / SimModule.WHEEL_CIRCUMFERENCE;

		return rotations / (2 * Math.PI);
	}

	@Override
	public Rotation2d getAngle() {
		return new Rotation2d(getRadians());
	}

	@Override
	protected void setVoltage(double voltage) {
		turn.setInputVoltage(voltage);	
	}

	@Override
	protected void reset() {
		encoderSim.setCount(0);
	}
}
