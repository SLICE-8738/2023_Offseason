// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateSequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotState;
import frc.robot.commands.GoToState;
import frc.robot.commands.Intake.OutTakeCommand;
import frc.robot.commands.StowCommands.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeAndStowCommandsSequence extends SequentialCommandGroup {
  /** Creates a new OuttakeAndStowCommandsSequence. */
  public OuttakeAndStowCommandsSequence(Intake intake, Arm arm, Elevator elevator, RobotState robotState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Initializing and defining the commands needed for the command sequence.
    GoToState goToState = new GoToState(elevator, arm, robotState);
    ParallelRaceGroup outtake = new OutTakeCommand(intake).withTimeout(1);
    Stow stow = new Stow(elevator, arm, intake);

    addCommands(
      goToState,
      outtake,
      stow
    );

  }

}
