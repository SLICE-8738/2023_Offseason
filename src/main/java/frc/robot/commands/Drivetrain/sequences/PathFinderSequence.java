// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.TrajectoryCommands;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathFinderSequence extends SequentialCommandGroup {
  /** Creates a new PathFinderSequence. */
  public PathFinderSequence(Drivetrain drive, Pose2d targetPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    InstantCommand initializePathfinding = new InstantCommand(Pathfinding::ensureInitialized);
    PathfindHolonomic pathFind = TrajectoryCommands.generatePathFindCommand(drive, targetPose);
    SetField2dCommand setField2d = new SetField2dCommand(
      new PathPlannerTrajectory(
        Pathfinding.getCurrentPath(
          Constants.kAutonomous.kPathConstraints, 
          new GoalEndState(
            0, 
            targetPose.getRotation())), 
          new ChassisSpeeds()), 
      drive);

    addCommands(
      initializePathfinding,
      pathFind,
      setField2d
    );
  }
}
