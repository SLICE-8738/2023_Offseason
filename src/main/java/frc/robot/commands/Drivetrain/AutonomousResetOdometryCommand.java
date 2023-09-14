// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class AutonomousResetOdometryCommand extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final PathPlannerState m_initialState;
  
   /** Creates a new ResetOdometryCommand. */
  public AutonomousResetOdometryCommand(Drivetrain drivetrain, PathPlannerState initialState) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_initialState = initialState;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetOdometry(new Pose2d(m_initialState.poseMeters.getTranslation(), m_initialState.holonomicRotation));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return true;
    
  }

}
