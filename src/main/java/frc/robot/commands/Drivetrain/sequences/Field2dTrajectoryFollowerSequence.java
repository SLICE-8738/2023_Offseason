// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoPaths;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.commands.Drivetrain.SetVisionImplementationCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Field2dTrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new Field2dTrajectoryFollowerSequence without reseting the position of the robot. */
  public Field2dTrajectoryFollowerSequence(Drivetrain drive, AutoPaths autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetVisionImplementationCommand preventVisionImplementation = new SetVisionImplementationCommand(drive, true);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    RamseteCommand ramseteCommand = AutoPaths.generateRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetVisionImplementationCommand allowVisionImplementation = new SetVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      setField2dCommand,
      ramseteCommand,
      stopDriveCommand,
      allowVisionImplementation);
      
  }

  /** Creates a new Field2dTrajectoryFollowerSequence, reseting the position of the robot at the beginning of the sequence. */
  public Field2dTrajectoryFollowerSequence(Drivetrain drive, AutoPaths autoPath, Pose2d position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetVisionImplementationCommand preventVisionImplementation = new SetVisionImplementationCommand(drive, true);
    ResetOdometryCommand resetOdometry = new ResetOdometryCommand(drive, position);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    RamseteCommand ramseteCommand = AutoPaths.generateRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetVisionImplementationCommand allowVisionImplementation = new SetVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      resetOdometry,
      setField2dCommand,
      ramseteCommand,
      stopDriveCommand,
      allowVisionImplementation);
      
  }

  /** Creates a new Field2dTrajectoryFollowerSequence to be used with either a generated double substation or node alignment trajectory. */
  public Field2dTrajectoryFollowerSequence(Drivetrain drive, Trajectory trajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetVisionImplementationCommand preventVisionImplementation = new SetVisionImplementationCommand(drive, true);
    ResetOdometryCommand correctOdometry = new ResetOdometryCommand(drive, Limelight.getLastBotPoseBlue());
    SetField2dCommand setField2dCommand = new SetField2dCommand(trajectory, drive);
    RamseteCommand ramseteCommand = AutoPaths.generateRamseteCommand(trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetVisionImplementationCommand allowVisionImplementation = new SetVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      correctOdometry,
      setField2dCommand,
      ramseteCommand,
      stopDriveCommand,
      allowVisionImplementation
    );

  }

}
