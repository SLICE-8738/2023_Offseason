// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import com.pathplanner.lib.path.PathPlannerPath;


import frc.robot.auto.AutoPath;

/** Add your docs here. */
public class TestPath extends AutoPath {

    public TestPath() {

        path = PathPlannerPath.fromPathFile("Test Trajectory");

    }

}
