package com.stuypulse.robot.subsystems.swerve.sim;

import com.stuypulse.robot.constants.SimModule;
import com.stuypulse.robot.constants.SimModule.Turn;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.robot.util.MotorSim;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimTurnControl extends TurnControl {
	private final MotorSim sim;

	public SimTurnControl() {
		super(Turn.Feedback.getController());

		sim = new MotorSim(Turn.Feedforward.getPlant(), Turn.Encoder.GEAR_RATIO);
		sim.setDistancePerRotation(SimModule.WHEEL_DIAMETER);
	}

	private double getRadians() {
		double rotations = sim.getDistanceMeters() / SimModule.WHEEL_CIRCUMFERENCE;

		return rotations / (2 * Math.PI);
	}

	@Override
	public Rotation2d getAngle() {
		return new Rotation2d(getRadians());
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
