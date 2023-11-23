// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.AutoPath;
import frc.robot.auto.AutoSelector;

/** Add your docs here. */
public class GamePieceToChargeStationPath extends AutoPath {

    public GamePieceToChargeStationPath(AutoSelector.StartingPosition startPosition) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Blue Left Game Piece To Charge Station");
                break;
            case BLUE_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Blue Middle Game Piece To Charge Station");
                break;
            case BLUE_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Blue Right Game Piece To Charge Station");
                break;
            case RED_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Red Left Game Piece To Charge Station");
                break;
            case RED_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Red Middle Game Piece To Charge Station");
                break;
            case RED_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Red Right Game Piece To Charge Station");
                break;
            default:
                break;

        }

    }

}
