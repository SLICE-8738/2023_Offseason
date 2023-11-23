// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.paths.TestPath;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectoryMode extends SequentialCommandGroup {
  /** Creates a new TestTrajectoryMode. */
  public TestTrajectoryMode(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    TestPath testPath = new TestPath();

    WaitCommand wait = new WaitCommand(1);
    Field2dTrajectoryFollowerSequence trajectory = new Field2dTrajectoryFollowerSequence(drive, testPath, testPath.getTrajectory().getInitialTargetHolonomicPose());

    addCommands(
      wait,
      trajectory
    );
  }
}
