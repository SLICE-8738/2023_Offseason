// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmGoToPosition extends Command {
  private final Arm m_arm;
  private final double m_elbowPosition, m_wristPosition;

  boolean goingOut;
  boolean secondary;

  /** Creates a new ArmGoToPosition. */
  public ArmGoToPosition(Arm arm, double elbowPosition, double wristPosition) {
    m_arm = arm;
    m_elbowPosition = elbowPosition;
    m_wristPosition = wristPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goingOut = (m_elbowPosition > m_arm.getElbowPosition());
    secondary = false;

    if (goingOut) {
      m_arm.setArmController(m_elbowPosition);
    }else {
      m_arm.setWristController(m_wristPosition);
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (secondary) {
      return;
    }

    if (goingOut) {
      // If going out, wait until the elbow if far enough out before moving the wrist
      if (.05 + getY(m_arm.getElbowPosition(), m_wristPosition) < getX(m_arm.getElbowPosition(), m_wristPosition)) {
        m_arm.setWristController(m_wristPosition);
        secondary = true;
      }
    }else {
      // If going in, wait until the wrist is far enough out before moving the elbow
      if (.05 + getY(m_elbowPosition, m_arm.getWristPosition()) < getX(m_elbowPosition, m_arm.getWristPosition())) {
        m_arm.setArmController(m_elbowPosition);
        secondary = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.isElbowAtTarget() && m_arm.isWristAtTarget();
  }

  /**
   * Returns the x coordinate of the end effector, given the elbow and wrist angles
   * (0,0) is the connection of the lower arm to the elevator, and the x axis is horizontal
   * @param elbowPosition the angle of the elbow, in degrees, from vertical + out - in
   * @param wristPosition the angle of the wrist, in degrees, from parallel with the upper arm + out - in
   * @return the x coordinate of the end effector, in meters
   */
  private double getX(double elbowPosition, double wristPosition) {
    double u = Constants.kArm.UPPER_ARM_LENGTH;
    double l = Constants.kArm.LOWER_ARM_LENGTH;
    double w = Constants.kArm.WRIST_LENGTH;
    return l + u * Math.sin(Math.toRadians(elbowPosition)) + w * Math.sin(Math.toRadians(elbowPosition + wristPosition));
  }

  /**
   * Returns the y coordinate of the end effector, given the elbow and wrist angles
   * (0,0) is the connection of the lower arm to the elevator, and the y axis is vertical
   * @param elbowPosition the angle of the elbow, in degrees, from vertical + out - in
   * @param wristPosition the angle of the wrist, in degrees, from parallel with the upper arm + out - in
   * @return the y coordinate of the end effector, in meters
   */
  private double getY(double elbowPosition, double wristPosition) {
    double u = Constants.kArm.UPPER_ARM_LENGTH;
    double w = Constants.kArm.WRIST_LENGTH;
    return u * Math.cos(Math.toRadians(elbowPosition)) + w * Math.cos(Math.toRadians(elbowPosition + wristPosition));
  }
}
