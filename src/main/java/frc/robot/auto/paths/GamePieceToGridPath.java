package frc.robot.auto.paths;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.AutoPath;
import frc.robot.auto.AutoSelector;

public class GamePieceToGridPath extends AutoPath {

    public GamePieceToGridPath(AutoSelector.StartingPosition startPosition) {

        switch (startPosition) {

            case BLUE_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Blue Left Game Piece To Grid");
                break;
            case BLUE_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Blue Middle Game Piece To Grid");
                break;
            case BLUE_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Blue Right Game Piece To Grid");
                break;
            case RED_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Red Left Game Piece To Grid");
                break;
            case RED_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Red Middle Game Piece To Grid");
                break;
            case RED_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Red Right Game Piece To Grid");
                break;
            default:
                break;

        }

    }

}
