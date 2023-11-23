// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.AutoPath;
import frc.robot.auto.AutoSelector;

/** Add your docs here. */
public class GridOutOfCommunityToChargeStationPath extends AutoPath {

    public GridOutOfCommunityToChargeStationPath(AutoSelector.StartingPosition startPosition) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Blue Left Grid Out Of Community To Charge Station");
                break;
            case BLUE_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Blue Middle Grid Out Of Community To Charge Station");
                break;
            case BLUE_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Blue Right Grid Out Of Community To Charge Station");
                break;
            case RED_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Red Left Grid Out Of Community To Charge Station");
                break;
            case RED_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Red Middle Grid Out Of Community To Charge Station");
                break;
            case RED_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Red Right Grid Out Of Community To Charge Station");
                break;
            default:
                break;

        }

    }

}
