// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

public class HoldCubeCommand extends CommandBase {

  private final Intake m_intake;

  /** Creates a new HoldCubeCommand. */
  public HoldCubeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

    m_intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Arm.stowState == StowState.Cube) {

      

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
