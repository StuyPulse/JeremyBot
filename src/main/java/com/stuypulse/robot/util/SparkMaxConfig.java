package com.stuypulse.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.stuypulse.robot.constants.Settings;

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

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500 / Settings.UPDATE_RATE);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000 / Settings.UPDATE_RATE);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000 / Settings.UPDATE_RATE);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 2500 / Settings.UPDATE_RATE);

        motor.burnFlash();
    }

}
