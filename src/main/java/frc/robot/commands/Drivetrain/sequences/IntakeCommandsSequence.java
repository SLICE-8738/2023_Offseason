// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoToState;
import frc.robot.commands.Intake.IntakeCommand;
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
  public IntakeCommandsSequence(Intake m_intake, Arm m_Arm, Elevator m_Elevator) {

    // Initializing and defining the commands needed for the command sequence.
    GoToState goToState = new GoToState(m_Elevator, m_Arm, Constants.kRobotStates.stow);
    IntakeCommand intakeCommand = new IntakeCommand(m_intake, m_Arm);
    // Creating instant commands that can be defined in this command
    new InstantCommand(() -> this.setCube());
    new InstantCommand(() -> this.setCone());
    //new InstantCommand(() -> this.setState());        might use this to make logic better

    /*runs a different sequential command depending on what game piece needs to be stowed
     * the comments are placeholders currently and will be changed to the proper logic once able to */
    if(/*gamepiece is cone*/null){
      addCommands(setCone(), goToState, intakeCommand.repeatedly());
    }
    else if(/*gamepiece is cube */ null){
      addCommands(setCube()), goToState, intakeCommand.repeatedly();
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

  /* might use
  public void setState(){

  }*/

}

