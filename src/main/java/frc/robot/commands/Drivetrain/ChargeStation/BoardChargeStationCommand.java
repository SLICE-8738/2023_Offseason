// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.ChargeStation;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

//import edu.wpi.first.wpilibj.smartdashboard.*;

/** An example command that uses an example subsystem. */
public class BoardChargeStationCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private double maxAngle = 0, angle = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BoardChargeStationCommand(Drivetrain drivetrain) {
    this.m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    /**
     * THIS SHOULD BE CHANGED TO getPitch()
     * IF THE PITCH AXIS IS PARALLEL TO THE ROBOT POINTING FORWARD 
     */
    angle = m_drivetrain.getRoll();

    maxAngle = angle;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Checks if current angle is the largest angle

    /**
     * THIS SHOULD BE CHANGED TO getPitch()
     * IF THE PITCH AXIS IS PARALLEL TO THE ROBOT POINTING FORWARD 
     */
    angle = m_drivetrain.getRoll();

    maxAngle = Math.max(maxAngle, angle);

    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(Constants.kDrivetrain.BOARD_CHARGE_SPEED, 0), new Rotation2d()), true, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(), new Rotation2d()), true, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    /**
     * THIS SHOULD BE CHANGED TO getPitch()
     * IF THE PITCH AXIS IS PARALLEL TO THE ROBOT POINTING FORWARD 
     */
    double angle = Math.abs(m_drivetrain.getRoll());
    
    boolean dropped = maxAngle - angle > Constants.kDrivetrain.BOARD_CHARGE_ANGLE_CHANGE_THRESHOLD;
    boolean aboveThreshold = maxAngle > Constants.kDrivetrain.BOARD_CHARGE_MINIMUM_STOP_ANGLE;
    return dropped && aboveThreshold;

  }
}
