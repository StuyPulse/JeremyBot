package com.stuypulse.robot.constants;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;

public interface Settings {
    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
}
