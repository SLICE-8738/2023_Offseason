// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.TrajectoryCommands;
import frc.robot.auto.AutoPath;
//import frc.robot.commands.Drivetrain.PrepareAutoRotationsCommand;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new TrajectoryFollowerSequence to be used with an {@link AutoPath} that does not reset the position of the robot. */
  public TrajectoryFollowerSequence(Drivetrain drive, AutoPath autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, autoPath.trajectory);
    PPSwerveControllerCommand swerveControllerCommand = TrajectoryCommands.generatePPSwerveControllerCommand(drive, autoPath.trajectory, true);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      //prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

  /** Creates a new TrajectoryFollowerSequence to be used with an {@link AutoPath} that resets the position of the robot at the beginning of the sequence. */
  public TrajectoryFollowerSequence(Drivetrain drive, AutoPath autoPath, Pose2d position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(drive, position);
    //PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, autoPath.trajectory);
    PPSwerveControllerCommand swerveControllerCommand = TrajectoryCommands.generatePPSwerveControllerCommand(drive, autoPath.trajectory, true);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      resetOdometryCommand,
      //prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

  /** Creates a new TrajectoryFollowerSequence to be used with a trajectory generated by the {@link PathPlanner} class. */
  public TrajectoryFollowerSequence(Drivetrain drive, PathPlannerTrajectory trajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //SetVisionImplementationCommand preventVisionImplementation = new SetVisionImplementationCommand(drive, true);
    PPSwerveControllerCommand swerveControllerCommand = TrajectoryCommands.generatePPSwerveControllerCommand(drive, trajectory, false);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    //SetVisionImplementationCommand allowVisionImplementation = new SetVisionImplementationCommand(drive, false);

    addCommands(
      //preventVisionImplementation,
      swerveControllerCommand,
      stopDriveCommand/*,
      allowVisionImplementation*/
    );

  }

}
