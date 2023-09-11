// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StowCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

public class Test extends CommandBase {
  /** Creates a new Test. */
  private Arm arm;
  private Intake intake;
  private Timer timer;
  private boolean tested = false;                                     // Keeps track of if execute has checked stowState
  
  public Test(Arm armSubsystem, Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    
    arm = armSubsystem;
    intake = intakeSubsystem;
    addRequirements(armSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();                                               
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Arm.stowState == StowState.Cube) { 
      tested = true; // Assume you have a cube                      // Checks the current stowState's motor, and ends the command if output is greater than the threshold
    //arm.wristSecureCube();    
      if (arm.getArmOutputCurrent()>Constants.kArm.CUBE_THRESHOLD) {
        tested = true;
      }
    } else if (Arm.stowState == StowState.Cone) {
      tested = true; // Assume you have a cone
    intake.IntakeSpinHoldUp();
      if (arm.getWristOutputCurrent()>Constants.kIntake.CONE_THRESHOLD) {
        tested = true;
      } 
    } else {                                                        // Ends the command if the current stowState is Nothing.
      tested = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {                            // Sets the stowState to Nothing, if the command ends via timer.
    if (timer.get()>1) {
      Arm.stowState = StowState.Nothing;
    }
    arm.wristReleaseCube();                                         // Stops run motors
    intake.IntakeStopSpinning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {                                     // End if execute checked the stowState, or if 1 second(s) has passed
    if (tested) {
      return true;
    } else {
      return (timer.get()>1);
    }
  }
}
