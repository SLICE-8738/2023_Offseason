package frc.robot.auto.paths;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.AutoPath;
import frc.robot.auto.AutoSelector;


public class GridToGamePiecePath extends AutoPath {

    public GridToGamePiecePath(AutoSelector.StartingPosition startPosition) {

        switch (startPosition) {

            case BLUE_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Blue Left Grid To Game Piece");
                break;
            case BLUE_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Blue Middle Grid To Game Piece");
                break;
            case BLUE_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Blue Right Grid To Game Piece");
                break;
            case RED_COMMUNITY_LEFT:
                path = PathPlannerPath.fromPathFile("Red Left Grid To Game Piece");
                break;
            case RED_COMMUNITY_MIDDLE:
                path = PathPlannerPath.fromPathFile("Red Middle Grid To Game Piece");
                break;
            case RED_COMMUNITY_RIGHT:
                path = PathPlannerPath.fromPathFile("Red Right Grid To Game Piece");
                break;
            default:
                break;
        }

    }

}
