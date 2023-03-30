// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class InstantCalibrationCommand extends CommandBase {
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  /** Creates a new InstantCalibrationCommand. */
  public InstantCalibrationCommand(Elevator elevator, Wrist wrist) {
    m_elevator = elevator;
    m_wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setEncoderPosition(0);
    m_wrist.setEncoder(Constants.Wrist.MIN_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
