/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;
import com.stuypulse.stuylib.network.limelight.Limelight.LEDMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.stuypulse.robot.constants.Settings;

public class Camera extends SubsystemBase {

    private final Limelight limelight;

    public Camera() {
        this.limelight = Limelight.getInstance();

        for (int port : Settings.Limelight.PORTS) {
            PortForwarder.add(port, "limelight.local", port);
        }

        CameraServer.startAutomaticCapture();
        // CameraServer.startAutomaticCapture("Intake Camera", 0);
        // CameraServer.startAutomaticCapture("Intake Camera", 1);
    }

    /*** Has Target ***/
    public boolean hasAnyTarget() {
        return limelight.getValidTarget();
    }

    public boolean hasTarget() {
        return hasAnyTarget()
                && Settings.Limelight.MIN_VALID_DISTANCE < getDistance()
                && getDistance() < Settings.Limelight.MAX_VALID_DISTANCE;
    }

    /*** Angles ***/
    public Angle getXAngle() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable To Find Target! [getXAngle() was called]");
            return Angle.kZero;
        }

        return Angle.fromDegrees(
                limelight.getTargetXAngle() + Settings.Limelight.LIMELIGHT_YAW.get());
    }

    public Angle getYAngle() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable To Find Target! [getYAngle() was called]");
            return Angle.kZero;
        }

        return Angle.fromDegrees(
                limelight.getTargetYAngle() + Settings.Limelight.LIMELIGHT_PITCH.get());
    }

    /*** Distance ***/

    /** returns distance from intake to center of hub */
    public double getDistance() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable To Find Target! [getDistance() was called]");
            return Settings.Limelight.RING_DISTANCE.get();
        }

        return Settings.Limelight.CENTER_TO_HUB
                + Settings.Limelight.LIMELIGHT_TO_INTAKE
                + Settings.Limelight.HEIGHT_DIFFERENCE
                        / getYAngle().tan(); // distance from edge of goal to limelight
    }

    /*** Periodic ***/

    @Override
    public void periodic() {
        if (!limelight.isConnected()) {
            Settings.reportWarning("Limelight Disconnected!");
        }

        if (DriverStation.isDisabled()) {
            limelight.setLEDMode(LEDMode.PIPELINE);
        } else {
            limelight.setLEDMode(LEDMode.FORCE_ON);
        }
    }
}