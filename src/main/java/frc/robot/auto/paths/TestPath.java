// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import frc.robot.auto.AutoPath;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

/** Add your docs here. */
public class TestPath extends AutoPath {

    public TestPath() {

        trajectory = PathPlanner.loadPath("Test Trajectory", new PathConstraints(3, 2));

    }

}
