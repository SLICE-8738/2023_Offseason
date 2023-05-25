package frc.robot.auto.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.auto.AutoPath;
import frc.robot.auto.AutoSelector;

public class GamePieceToGridPath implements AutoPath {

    Trajectory trajectory;
    
    public GamePieceToGridPath(AutoSelector.StartingPosition startPosition) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Blue Left Game Piece To Grid", new PathConstraints(0.5, 0.2));
                break;
            case BLUE_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Blue Middle Game Piece To Grid", new PathConstraints(0.5, 0.2));
                break;
            case BLUE_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Blue Right Game Piece To Grid", new PathConstraints(0.5, 0.2));
                break;
            case RED_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Red Left Game Piece To Grid", new PathConstraints(0.5, 0.2));
                break;
            case RED_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Red Middle Game Piece To Grid", new PathConstraints(0.5, 0.2));
                break;
            case RED_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Red Right Game Piece To Grid", new PathConstraints(0.5, 0.2));
                break;
            default:
                break;

        }

    }

    @Override
    public Trajectory getTrajectory() {

        return trajectory;

    }

}
