package com.stuypulse.robot.subsystems.modules.implementations.statespace;

import com.stuypulse.robot.subsystems.modules.DriveControl;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;

public class StateSpaceDriveControl extends DriveControl {

    private final LinearSystemLoop<N1, N1, N1> loop;
  
    public StateSpaceDriveControl(PhysicalControl physicalControl, LinearSystemLoop<N1, N1, N1> loop) {
        super(physicalControl);
        this.loop = loop;
    }

    public void setVelocity(double velocity) {
        loop.setNextR(velocity);
    }

    @Override
    public void periodic() {
        loop.correct(VecBuilder.fill(getVelocity()));
        loop.predict(0.020);

        double outputVolts = loop.getU(0);

        setVoltage(outputVolts);
    }
}
