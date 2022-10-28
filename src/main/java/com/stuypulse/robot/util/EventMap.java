package com.stuypulse.robot.util;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;

public class EventMap {

	public static class Marker {
		public final String name;
		public final Command command;

		public Marker(String name, Command command) {
			this.name = name;
			this.command = command;
		}
	}

	public static HashMap<String, Command> getEventMap(Marker... events) {
		HashMap<String, Command> map = new HashMap<>();

		for (Marker e : events) {
			map.put(e.name, e.command);
		}

		return map;
	}
	
}
