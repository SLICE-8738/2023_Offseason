// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.TrajectoryCommands;
import frc.robot.auto.AutoPath;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Field2dTrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new Field2dTrajectoryFollowerSequence to be used with an {@link AutoPath} that does not reset the position of the robot. */
  public Field2dTrajectoryFollowerSequence(Drivetrain drive, AutoPath autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.getTrajectory(), drive);
    FollowPathHolonomic swerveControllerCommand = TrajectoryCommands.generateFollowPathHolonomicCommand(drive, autoPath.path);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      setField2dCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

  /** Creates a new Field2dTrajectoryFollowerSequence to be used with an {@link AutoPath} that resets the position of the robot at the beginning of the sequence. */
  public Field2dTrajectoryFollowerSequence(Drivetrain drive, AutoPath autoPath, Pose2d initialPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(drive, initialPose);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.getTrajectory(), drive);
    FollowPathHolonomic swerveControllerCommand = TrajectoryCommands.generateFollowPathHolonomicCommand(drive, autoPath.path);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      resetOdometryCommand,
      setField2dCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

}