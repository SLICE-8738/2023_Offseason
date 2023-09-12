// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.robot.auto.AutoPath;
import frc.robot.auto.AutoSelector;

/** Add your docs here. */
public class GamePieceToChargeStationPath extends AutoPath {

    public GamePieceToChargeStationPath(AutoSelector.StartingPosition startPosition) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Blue Left Game Piece To Charge Station", new PathConstraints(0.5, 0.2));
                break;
            case BLUE_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Blue Middle Game Piece To Chage Station", new PathConstraints(0.5, 0.2));
                break;
            case BLUE_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Blue Right Game Piece To Charge Station", new PathConstraints(3.0, 2.0));
                break;
            case RED_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Red Left Game Piece To Charge Station", new PathConstraints(3.0, 2.0));
                break;
            case RED_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Red Middle Game Piece To Charge Station", new PathConstraints(0.5, 0.2));
                break;
            case RED_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Red Right Game Piece To Charge Station", new PathConstraints(0.5, 0.2));
                break;
            default:
                break;

        }

    }

}
