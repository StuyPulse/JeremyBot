package com.stuypulse.robot.subsystems.modules.implementations.statespace;

import com.stuypulse.robot.subsystems.modules.TurnControl;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;

public class StateSpaceTurnControl extends TurnControl {

    private final LinearSystemLoop<N2, N1, N1> loop;
  
    public StateSpaceTurnControl(PhysicalControl physicalControl, LinearSystemLoop<N2, N1, N1> loop) {
        super(physicalControl);
        this.loop = loop;
    }

    public void setAngle(Rotation2d target) {
        loop.setNextR(target.getRadians());
    }

    @Override
    public void periodic() {
        loop.correct(VecBuilder.fill(getAngle().getRadians()));
        loop.predict(0.020);

        double outputVolts = loop.getU(0);

        setVoltage(outputVolts);
    }
}
