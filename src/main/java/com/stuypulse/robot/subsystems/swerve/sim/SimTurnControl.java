package com.stuypulse.robot.subsystems.swerve.sim;

import com.stuypulse.robot.constants.SimModule.Turn;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimTurnControl extends TurnControl {

    private LinearSystemSim<N2, N1, N1> position;
    private StopWatch timer;

    private double voltage;
    private double offset;

	public SimTurnControl() {
		super(Turn.Feedback.getController());
        position = new LinearSystemSim<>(Turn.Feedforward.getPlant());
        voltage = 0.0;
        offset = 0.0;

        timer = new StopWatch();
	}

    private double getRawRadians() {
        return position.getOutput(0);
    }

	@Override
	public Rotation2d getAngle() {
		return new Rotation2d(getRawRadians() - offset);
	}

	@Override
	protected void setVoltage(double voltage) {
        this.voltage = voltage;
	}

	@Override
	protected void reset() {
        offset = getRawRadians();
	}

    @Override
    protected void log(String id) {
        super.log(id);

        SmartDashboard.putNumber(id + "/Turn Voltage", voltage);
    }
	
    @Override
    public void periodic() {
        super.periodic();
        position.setInput(voltage);
        position.update(timer.reset());
    }
}
