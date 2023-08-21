// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

public class OutTakeCommand extends CommandBase {
  /** Creates a new OutTakeCommand. */
  private Intake intake; 

  public OutTakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Will initialize when a button is pressed 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Checks to see if it's a cone or cube being spit out and spins in the appropriate direction. 
    //"cube" is a place holder
    if (Arm.stowState == StowState.Cube){
      intake.IntakeSpinOut(); 
    }
    //"cone" is a place holder 
    else if (Arm.stowState == StowState.Cone){
      intake.IntakeSpinUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //When button is stoppedPressed
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
