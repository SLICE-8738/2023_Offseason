// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.commands.Drivetrain.TrajectoryFollowerSequence;
import frc.robot.commands.Indexer.AutoShoot.IndexerDownSlightCommand;
import frc.robot.commands.Intake.IntakeSchedulableCommand;
import frc.robot.commands.Shooter.AutoShoot.SmartAutoShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class pathWeaver3BallAutoRoutine extends SequentialCommandGroup {

  /** Creates a new pathWeaverAutoRoutine. */
  public pathWeaver3BallAutoRoutine(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight, Joystick leftJoystick, Joystick rightJoystick) {

    TrajectoryFollowerSequence path1TrajectoryForward = new TrajectoryFollowerSequence(drivetrain, Paths.getAutoPath(1), Paths.returnAutoTrajectory().getInitialPose());
    ParallelDeadlineGroup path1ForwardGroup = new ParallelDeadlineGroup(path1TrajectoryForward, new IntakeSchedulableCommand(intake, true));

    TrajectoryFollowerSequence path2TrajectoryForward = new TrajectoryFollowerSequence(drivetrain, Paths.getAutoPath(2), Paths.returnAutoTrajectory().getInitialPose());
    ParallelDeadlineGroup path2ForwardGroup = new ParallelDeadlineGroup(path2TrajectoryForward, new IntakeSchedulableCommand(intake, true));

    TrajectoryFollowerSequence path3TrajectoryForward = new TrajectoryFollowerSequence(drivetrain, Paths.getAutoPath(3), Paths.returnAutoTrajectory().getInitialPose());

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autoPathWidgetCommand(),
      new IndexerDownSlightCommand(indexer),
      path1ForwardGroup,
      new smart2BallShootSequence(indexer, intake, shooter, drivetrain, limelight, leftJoystick, rightJoystick),
      path2ForwardGroup,
      new IndexerDownSlightCommand(indexer),
      path3TrajectoryForward,
      new SmartAutoShoot(limelight, shooter)
    );

  }
  
}
