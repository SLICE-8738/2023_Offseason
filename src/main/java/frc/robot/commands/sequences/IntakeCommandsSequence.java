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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** This sequential command group is logic for determining what state the intake should go to */
public class IntakeCommandsSequence extends SequentialCommandGroup {
  
  
  /** Creates a new IntakeCommandsSequence. */
  public IntakeCommandsSequence(Intake m_intake, Arm m_Arm, Elevator m_Elevator, StowState stowState, RobotState m_RobotState) {

    // Initializing and defining the commands needed for the command sequence.
    GoToState goToState = new GoToState(m_Elevator, m_Arm, m_RobotState);
    IntakeCommand intakeCommand = new IntakeCommand(m_intake, m_Arm);

    ParallelRaceGroup outSlight = new OutTakeCommand(m_intake).withTimeout(0.1);

    InstantCommand setGamePieceSecured = new InstantCommand(() -> m_intake.setGamePieceSecured(true));
    

    /* runs a different sequential command depending on what game piece needs to be stowed
     * the instant commands are used to set the StowState of the Arm based on what game piece
     * has been picked up by the robot.
     * The robot then goes to the desired state and runs intakeCommand
     */
    if( stowState == StowState.Cone ){
      addCommands(
        new InstantCommand(() -> this.setCone()), 
        new GoToState(m_Elevator, m_Arm, m_RobotState), 
        new IntakeCommand(m_intake, m_Arm), 
        new OutTakeCommand(m_intake).withTimeout(0.1), 
        new InstantCommand(() -> m_intake.setGamePieceSecured(true)));
    }
    else if( stowState == StowState.Cube ){
      addCommands(
        new InstantCommand(() -> this.setCube()), 
        new GoToState(m_Elevator, m_Arm, m_RobotState), 
        new IntakeCommand(m_intake, m_Arm), 
        new OutTakeCommand(m_intake).withTimeout(0.1), 
        new InstantCommand(() -> m_intake.setGamePieceSecured(true)));
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

