// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StowCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.StowState;
import frc.robot.subsystems.Intake;


public class Hold extends CommandBase {
  private Intake intake;
  /** Creates a new Hold. */
  public Hold(Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Arm.stowState == StowState.Nothing) {                       // Runs the motor for given stowState to hold in place

    } else if (Arm.stowState == StowState.Cube) {
        intake.IntakeRetainCube();
    } else if (Arm.stowState == StowState.Cone) {
        intake.IntakeSpinHoldUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.IntakeStopSpinning();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
