// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickFilter;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends CommandBase {
  private final Elevator m_elevator;
  private final GenericHID m_controller;
  private final JoystickFilter m_elevatorFilter;

  /** Creates a new ManualElevator. */
  public ManualElevator(Elevator elevator, GenericHID controller) {
    m_elevator = elevator;
    m_controller = controller;
    m_elevatorFilter = new JoystickFilter(0.07, 0.1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorSpeed = m_elevatorFilter.filter(m_controller.getRawAxis(1));
    m_elevator.runElevator(elevatorSpeed);
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
