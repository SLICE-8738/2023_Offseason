// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Button;
import frc.robot.JoystickFilter;
import frc.robot.subsystems.Arm;

public class ManualArm extends Command {
  private final Arm m_arm;
  private final GenericHID m_controller;
  private final JoystickFilter m_elbowFilter;

  private boolean elbowStill;

  /** Creates a new ManualArm. */
  public ManualArm(Arm arm, GenericHID controller) {
    m_arm = arm;
    m_controller = controller;
    m_elbowFilter = new JoystickFilter(0.07, 0.7);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbowStill = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elbowSpeed = m_elbowFilter.filter(m_controller.getRawAxis(3)) * 0.5;
    double wristSpeed = 0;
    if (Button.wristUp.getAsBoolean()) {
      wristSpeed = -0.6;
    } else if (Button.wristDown.getAsBoolean()) {
      wristSpeed = 0.4;
    }

    if (elbowSpeed == 0 && !elbowStill) {
      m_arm.setArmController(m_arm.getElbowPosition());
      elbowStill = true;
    }else if (elbowSpeed != 0) {
      m_arm.runElbow(elbowSpeed);
      elbowStill = false;
    }
    m_arm.runWrist(wristSpeed);
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
