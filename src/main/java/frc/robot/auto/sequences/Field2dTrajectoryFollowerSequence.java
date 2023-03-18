// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.commands.PrepareAutoRotationsCommand;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Field2dTrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new Field2dTrajectoryFollowerSequence without reseting the position of the robot. */
  public Field2dTrajectoryFollowerSequence(SwerveDrivetrain drive, AutoPaths autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, autoPath.trajectory);
    SwerveControllerCommand swerveControllerCommand = autoPath.generateSwerveControllerCommand(drive, autoPath.trajectory);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      setField2dCommand,
      prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

  /** Creates a new Field2dTrajectoryFollowerSequence, reseting the position of the robot at the beginning of the sequence. */
  public Field2dTrajectoryFollowerSequence(SwerveDrivetrain drive, AutoPaths autoPath, Pose2d position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(drive, position);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, autoPath.trajectory);
    SwerveControllerCommand swerveControllerCommand = autoPath.generateSwerveControllerCommand(drive, autoPath.trajectory);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      resetOdometryCommand,
      setField2dCommand,
      prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDriveCommand);
      
  }

}
