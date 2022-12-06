package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.util.SparkMaxConfig;

public interface Motors {
	SparkMaxConfig TURN_CONFIG = new SparkMaxConfig(false, IdleMode.kBrake, 20, 0.0);
	SparkMaxConfig DRIVE_CONFIG = new SparkMaxConfig(false, IdleMode.kBrake, 40, 0.0);
}
