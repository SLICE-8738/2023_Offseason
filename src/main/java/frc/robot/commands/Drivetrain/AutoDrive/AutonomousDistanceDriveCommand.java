// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.AutoDrive;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.Drivetrain.SetInitialPositionCommand;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

//import edu.wpi.first.wpilibj.smartdashboard.*;

/** 
 * This command should be used in Pathplannerless autonomous sequences to
 * drive the robot at given x and y speeds over given x and y distances.
 * 
 * <p> Instances of this command should be used along with a {@link SetInitialPositionCommand} in sequences if 
 * there is no other command resetting the odometry elsewhere in the sequence.
 */
public class AutonomousDistanceDriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private double xDistance, yDistance;
  private double xSpeed, ySpeed;

  private Translation2d targetTranslation;

  private double xError, yError;

  private final BooleanSupplier onBlueAlliance;
  private boolean onBlueAllianceFinal;

  /**
   * Creates a new AutonomousDistanceDriveCommand.
   *
   * @param drivetrain The instance of the Drivetrain subsystem declared in RobotContainer.
   * @param speeds The desired field-relative desired velocities in meters/second for the robot to move at along
   *               the X and Y axes of the field(forwards/backwards from driver POV).
   * @param distances The desired field-relative desired distances in meters along the X and Y axes of the field
   *                  for the robot to travel.
   * @param onBlueAlliance A boolean supplier that returns whether the robot is running autonomous on the blue alliance side
   *                       of the field.
   */
  public AutonomousDistanceDriveCommand(Drivetrain drivetrain, Translation2d speeds, Translation2d distances, BooleanSupplier onBlueAlliance) {
    m_drivetrain = drivetrain;
    this.onBlueAlliance = onBlueAlliance;

    xSpeed = speeds.getX();
    ySpeed = speeds.getY();

    xDistance = Math.abs(distances.getX()) * Math.signum(xSpeed);
    yDistance = Math.abs(distances.getY()) * Math.signum(ySpeed);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    onBlueAllianceFinal = onBlueAlliance.getAsBoolean();

    if(!onBlueAllianceFinal) {

      xSpeed *= -1;
      ySpeed *= -1;

      xDistance *= -1;
      yDistance *= -1;

    }

    Pose2d initialPosition = m_drivetrain.getPose();
    targetTranslation = new Translation2d(initialPosition.getX() + xDistance, initialPosition.getY() + yDistance);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPosition = m_drivetrain.getPose();

    xError = Math.abs(currentPosition.getX() - targetTranslation.getX()); 
    yError = Math.abs(currentPosition.getY() - targetTranslation.getY()); 

    if(xError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE) {

      xSpeed = 0;

    }
    if(yError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE) {

      ySpeed = 0;

    }

    // Sets the x speed and y speed of the robot
    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(xSpeed, ySpeed), new Rotation2d()), false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(), new Rotation2d()), false, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return 
      (xError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE) && 
      (yError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE);

  }
}