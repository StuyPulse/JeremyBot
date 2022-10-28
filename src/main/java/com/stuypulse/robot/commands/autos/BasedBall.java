package com.stuypulse.robot.commands.autos;

import java.util.HashMap;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.FollowTrajectory;
import com.stuypulse.robot.util.EventMap;
import com.stuypulse.robot.util.EventMap.Marker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BasedBall extends SequentialCommandGroup {

	private static final HashMap<String, Command> EVENT_MAP = EventMap.getEventMap(
		new Marker("marker", new InstantCommand(() -> System.out.println("MARKER")))
	);

	public BasedBall(RobotContainer robot) {
		addCommands(
			new FollowTrajectory(robot.swerve, "Based Ball", EVENT_MAP).robotRelative()
		);
	}
	
}
