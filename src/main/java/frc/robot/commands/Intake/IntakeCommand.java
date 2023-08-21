// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

public class IntakeCommand extends CommandBase {
  private Intake intake; 
  private Arm arm;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Will initialize when go to position is done
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Checks to see if it's a cone or cube being picked up and spins in the appropriate direction. 

    if (Arm.stowState == StowState.Cube){
      intake.IntakeSpinUp(); 
      arm.wristSecureCube();
    }

    else if (Arm.stowState == StowState.Cone){
      intake.IntakeSpinOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Ends when Stow is called by operator //

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
