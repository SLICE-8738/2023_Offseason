// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotState;
import frc.robot.commands.GoToState;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OutTakeCommand;
import frc.robot.commands.StowCommands.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** This sequential command group is logic for determining what state the intake should go to */
public class IntakeAndStowCommandsSequence extends SequentialCommandGroup {
  
  /** Creates a new IntakeCommandsSequence. */
  public IntakeAndStowCommandsSequence(Intake intake, Arm arm, Elevator elevator, StowState stowState, RobotState robotState) {

    // Initializing and defining the commands needed for the command sequence.
    GoToState goToState = new GoToState(elevator, arm, robotState);
    IntakeCommand intakeCommand = new IntakeCommand(intake);
    ParallelRaceGroup outSlight = new OutTakeCommand(intake).withTimeout(0.1);    
    Stow stow = new Stow(elevator, arm, intake);

    /* runs a different sequential command depending on what game piece needs to be stowed
     * the instant commands are used to set the StowState of the Arm based on what game piece
     * has been picked up by the robot.
     * The robot then goes to the desired state and runs intakeCommand
     */
    if(stowState == StowState.Cone) {
      addCommands(new InstantCommand(() -> this.setCone()) , goToState, intakeCommand, outSlight, stow);
    }
    else if(stowState == StowState.Cube) {
      addCommands(new InstantCommand(() -> this.setCube()) , goToState, intakeCommand, outSlight, stow);
    }
  }

  // sets the arm stowstate to Cube
  public void setCube(){
    Arm.stowState = StowState.Cube;
  }

  // sets the arm stowstate to Cone
  public void setCone(){
    Arm.stowState = StowState.Cone;
  }

}