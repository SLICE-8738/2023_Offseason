// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.Arm.ArmGoToPosition;
import frc.robot.commands.Elevator.ElevatorGoToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * A command that moves the elevator and arm to a given state.
 */
public class GoToState extends ParallelCommandGroup {
  /** Creates a new GoToPosition. */
  public GoToState(Elevator elevator, Arm arm, RobotState robotState) {
    // Stowed and going out more than 15 degrees
    ArmGoToPosition stowedArmSequenceG15 = new ArmGoToPosition(arm, robotState.getElbowAngle(), robotState.getWristAngle());
    SequentialCommandGroup stowedElevatorSequenceG15 = new ElevatorGoToPosition(elevator, robotState.getElevatorHeight()).beforeStarting(new WaitUntilCommand(() -> arm.getElbowPosition() > 10));
    ParallelCommandGroup stowedSequenceG15 = new ParallelCommandGroup(stowedArmSequenceG15, stowedElevatorSequenceG15);

    // Stowed and not going out more than 15 degrees
    ArmGoToPosition getTo15 = new ArmGoToPosition(arm, 15, robotState.getWristAngle());
    ElevatorGoToPosition stowedElevatorSequenceL15 = new ElevatorGoToPosition(elevator, robotState.getElevatorHeight());
    SequentialCommandGroup stowedArmSequenceL15 = new ArmGoToPosition(arm, robotState.getElbowAngle(), robotState.getWristAngle()).beforeStarting(new WaitUntilCommand(() -> elevator.getElevatorHeight() >= 0.235));
    SequentialCommandGroup stowedSequenceL15 = new SequentialCommandGroup(getTo15, new ParallelCommandGroup(stowedArmSequenceL15, stowedElevatorSequenceL15));

    // Stowed
    ConditionalCommand stowedSequence = new ConditionalCommand(stowedSequenceL15, stowedSequenceG15, () -> (arm.getElbowPosition() < 15 && robotState.getElbowAngle() < 15 && robotState.getElevatorHeight() > 0.25));

    ArmGoToPosition outArmPrimary = new ArmGoToPosition(arm, 15, robotState.getWristAngle());
    SequentialCommandGroup outElevator = new ElevatorGoToPosition(elevator, robotState.getElevatorHeight()).beforeStarting(new WaitUntilCommand(() -> arm.getElbowPosition() > 10));
    ArmGoToPosition outArmSecondary = new ArmGoToPosition(arm, robotState.getElbowAngle(), robotState.getWristAngle());
    SequentialCommandGroup outSequence = new SequentialCommandGroup(new ParallelCommandGroup(outElevator, outArmPrimary), outArmSecondary);

    ConditionalCommand conditional = new ConditionalCommand(stowedSequence, outSequence, () -> elevator.getElevatorHeight() < 0.25);

    addCommands(
      conditional
    );

  }
}
