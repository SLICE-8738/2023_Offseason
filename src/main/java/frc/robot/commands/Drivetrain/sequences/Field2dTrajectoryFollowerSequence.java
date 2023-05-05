// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoPaths;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.commands.Drivetrain.SetPreventVisionImplementationCommand;
import frc.robot.commands.Drivetrain.Lambda.LambdaRamseteCommand;
import frc.robot.commands.Drivetrain.Lambda.LambdaResetOdometryCommand;
import frc.robot.commands.Drivetrain.Lambda.LambdaSetField2dCommand;
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
    SetPreventVisionImplementationCommand preventVisionImplementation = new SetPreventVisionImplementationCommand(drive, true);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    RamseteCommand ramseteCommand = AutoPaths.generateRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetPreventVisionImplementationCommand allowVisionImplementation = new SetPreventVisionImplementationCommand(drive, false);

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
    SetPreventVisionImplementationCommand preventVisionImplementation = new SetPreventVisionImplementationCommand(drive, true);
    ResetOdometryCommand resetOdometry = new ResetOdometryCommand(drive, position);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    RamseteCommand ramseteCommand = AutoPaths.generateRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetPreventVisionImplementationCommand allowVisionImplementation = new SetPreventVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      resetOdometry,
      setField2dCommand,
      ramseteCommand,
      stopDriveCommand,
      allowVisionImplementation);
      
  }

   /** Creates a new Field2dTrajectoryFollowerSequence using either a double substation or node alignment trajectory generated by the Limelight class. */
   public Field2dTrajectoryFollowerSequence(Drivetrain drive, boolean isNodeTrajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetPreventVisionImplementationCommand preventVisionImplementation = new SetPreventVisionImplementationCommand(drive, true);
    LambdaResetOdometryCommand correctOdometry = new LambdaResetOdometryCommand(drive, Limelight::getLastBotPoseBlue);
    LambdaSetField2dCommand setField2dCommand = new LambdaSetField2dCommand(() -> Limelight.generateAlignmentTrajectory(isNodeTrajectory, drive.getPose()), drive);
    LambdaRamseteCommand lambdaRamseteCommand = AutoPaths.generateLambdaRamseteCommand(() -> Limelight.generateAlignmentTrajectory(isNodeTrajectory, drive.getPose()), drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetPreventVisionImplementationCommand allowVisionImplementation = new SetPreventVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      correctOdometry,
      setField2dCommand,
      lambdaRamseteCommand,
      stopDriveCommand,
      allowVisionImplementation
    );

  }

}
