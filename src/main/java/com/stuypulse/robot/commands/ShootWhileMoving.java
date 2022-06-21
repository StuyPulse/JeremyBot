// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.commands;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.math.interpolation.Interpolator;
import com.stuypulse.stuylib.math.interpolation.NearestInterpolator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootWhileMoving extends CommandBase {
  private final double dT = 0.05;

  private RobotContainer robot;

  private Interpolator distanceToRpm;

  public ShootWhileMoving(RobotContainer robot) {
    // TODO: find points
    distanceToRpm = new NearestInterpolator();

    this.robot = robot;
    addRequirements(robot.swerve);
  }

  @Override
  public void initialize() {
  }

  private Vector2D getPosition() {
    return new Vector2D(robot.swerve.getPose().getTranslation());
  }

  private Vector2D getVirtualPosition() {
    Vector2D velocity = robot.swerve.getVelocity();
    return getPosition().add(velocity.mul(dT));
  }

  private Pose2d getVirtualPose() {
    Vector2D position = getVirtualPosition();

    return new Pose2d(
        position.getTranslation2d(),
        Field.Hub.CENTER.sub(position).getAngle().getRotation2d());
  }

  @Override
  public void execute() {
    Pose2d virtual = getVirtualPose();
    double rpm = distanceToRpm.interpolate(virtual.getTranslation().getDistance(Field.Hub.CENTER2D));
    Rotation2d angle = virtual.getRotation();

    SmartDashboard.putNumber("While Moving/RPM", rpm);
    SmartDashboard.putNumber("While Moving/Angle", angle.getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
  }
}
