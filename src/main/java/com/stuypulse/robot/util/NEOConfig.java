package com.stuypulse.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public final class NEOConfig {
    
    public boolean inverted;
    public IdleMode idleMode;

    public int smartCurrentLimit;
    public double openLoopRampRate;

    public double positionConversion;
    public double velocityConversion;

    public NEOConfig() {
        inverted = false;
        idleMode = IdleMode.kCoast;
        smartCurrentLimit = 80;
        openLoopRampRate = 0.0;

        positionConversion = 1.0;
        velocityConversion = 1.0;
    }

    public NEOConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public NEOConfig setIdleMode(IdleMode idleMode) {
        this.idleMode = idleMode;
        return this;
    }

    public NEOConfig setSmartCurrentLimit(int smartCurrentLimit) {
        this.smartCurrentLimit = smartCurrentLimit;
        return this;
    }

    public NEOConfig setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
        return this;
    }

    public NEOConfig setPositionConversion(double positionConversion) {
        this.positionConversion = positionConversion;
        return this;
    }

    public NEOConfig setVelocityConversion(double velocityConversion) {
        this.velocityConversion = velocityConversion;
        return this;
    }

    public void setup(CANSparkMax motor) {
        motor.setInverted(inverted);
        motor.setIdleMode(idleMode);
        motor.setSmartCurrentLimit(smartCurrentLimit);
        motor.setOpenLoopRampRate(openLoopRampRate);

        motor.getEncoder().setPositionConversionFactor(positionConversion);
        motor.getEncoder().setVelocityConversionFactor(velocityConversion);

        motor.burnFlash();
    }

}
