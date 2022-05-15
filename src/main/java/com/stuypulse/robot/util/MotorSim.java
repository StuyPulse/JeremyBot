package com.stuypulse.robot.util;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates a motor and encoder
 */
public class MotorSim {
    private final FlywheelSim sim;
    private double voltage;
    private double distance;
    private double distancePerRotation;

    public MotorSim(LinearSystem<N1, N1, N1> plant, double gearing) {
        sim = new FlywheelSim(
            plant,
            DCMotor.getNEO(1),
            gearing
        );

        voltage = 0;
        distance = 0;
        distancePerRotation = 1;
    }


    public double getSpeedMetersPerSecond() {
        return sim.getAngularVelocityRPM() * distancePerRotation / 60.0;
    }

    public double getDistance() {
        return distance;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setDistancePerRotation(double d) {
        distancePerRotation = d;
    }

    public void update(double seconds) {
        sim.setInputVoltage(voltage);
        sim.update(seconds);

        distance += getSpeedMetersPerSecond() * seconds;
    }

    public void reset() {
        distance = 0;
    }

}
