// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.TrajectoryCommands;
import frc.robot.auto.AutoPath;
import frc.robot.commands.Drivetrain.PrepareAutoRotationsCommand;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new TrajectoryFollowerSequence without reseting the position of the robot. */
  public TrajectoryFollowerSequence(Drivetrain drive, AutoPath autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, autoPath.trajectory);
    SwerveControllerCommand swerveControllerCommand = TrajectoryCommands.generateSwerveControllerCommand(drive, autoPath.trajectory);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

  /** Creates a new TrajectoryFollowerSequence, reseting the position of the robot at the beginning of the sequence. */
  public TrajectoryFollowerSequence(Drivetrain drive, AutoPath autoPath, Pose2d position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(drive, position);
    PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, autoPath.trajectory);
    SwerveControllerCommand swerveControllerCommand = TrajectoryCommands.generateSwerveControllerCommand(drive, autoPath.trajectory);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      resetOdometryCommand,
      prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

}
