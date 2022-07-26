package com.stuypulse.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public final class SparkMaxConfig {
    
    public boolean inverted;
    public IdleMode idleMode;
    public int smartCurrentLimit;
    public double openLoopRampRate;

    public SparkMaxConfig(boolean inverted, IdleMode idleMode, int smartCurrentLimit, double openLoopRampRate) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.smartCurrentLimit = smartCurrentLimit;
        this.openLoopRampRate = openLoopRampRate;
    }

    public void config(CANSparkMax motor) {
        motor.setInverted(inverted);
        motor.setIdleMode(idleMode);
        motor.setSmartCurrentLimit(smartCurrentLimit);
        motor.setOpenLoopRampRate(openLoopRampRate);
        motor.burnFlash();
    }

}
