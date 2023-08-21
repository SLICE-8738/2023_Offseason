// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.TrajectoryCommands;
import frc.robot.commands.Drivetrain.PrepareAutoRotationsCommand;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectoryFollowerSequence extends SequentialCommandGroup {
  /** 
   * Creates a new TestTrajectoryFollowerSequence, consisting of a trajectory with the given transform.
   * Instances of this sequence should be used to test {@link SwerveControllerCommand SwerveControllerCommands}.
   */
  public TestTrajectoryFollowerSequence(Drivetrain drive, Transform2d trajectoryTransform) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Trajectory testTrajectory = TrajectoryCommands.generateTestTrajectory(drive, trajectoryTransform);

    SetField2dCommand setField2d = new SetField2dCommand(testTrajectory, drive);
    PrepareAutoRotationsCommand prepareAutoRotationsCommand = new PrepareAutoRotationsCommand(drive, testTrajectory);
    SwerveControllerCommand swerveControllerCommand = TrajectoryCommands.generateSwerveControllerCommand(drive, testTrajectory, true);
    InstantCommand stopDrive = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      setField2d,
      prepareAutoRotationsCommand,
      swerveControllerCommand,
      stopDrive);

  }

}
