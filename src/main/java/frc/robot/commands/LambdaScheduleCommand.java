// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Schedules the given commands when this command is initialized. Useful for forking off from
 * CommandGroups. Note that if run from a composition, the composition will not know about the
 * status of the scheduled commands, and will treat this command as finishing instantly.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class LambdaScheduleCommand extends CommandBase {
  private final Set<Supplier<Command>> m_toSchedule;

  /**
   * Creates a new ScheduleCommand that schedules the given commands when initialized.
   *
   * @param toSchedule the commands to schedule
   */
  @SafeVarargs
  public LambdaScheduleCommand(Supplier<Command>... toSchedule) {
    m_toSchedule = Set.of(toSchedule);
  }

  @Override
  public void initialize() {
    for (Supplier<Command> command : m_toSchedule) {
      command.get().schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
