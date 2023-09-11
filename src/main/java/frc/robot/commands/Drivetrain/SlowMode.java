// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SlowMode extends CommandBase {
  Drivetrain m_drivetrain;
  /** Creates a new SlowMode. */
  public SlowMode(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // THERE SHOULDN'T BE A REQUIREMENT FOR THIS COMMAND
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.speedPercent = 0.25;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.speedPercent = 1;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
