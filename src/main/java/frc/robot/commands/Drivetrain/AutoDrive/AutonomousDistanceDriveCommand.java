// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.AutoDrive;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.Drivetrain.SetInitialPositionCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * This command should be used in Pathplannerless autonomous sequences to
 * drive the robot at given x and y speeds over given x and y distances.
 * 
 * <p> Instances of this command should be used along with a {@link SetInitialPositionCommand} in sequences if 
 * there is no other command resetting the odometry elsewhere in the sequence.
 */
public class AutonomousDistanceDriveCommand extends CommandBase {

  private final Drivetrain m_drivetrain;

  private final double targetForwardDistance, targetSidewaysDistance;
  private double forwardSpeed, sidewaysSpeed;

  private SwerveModulePosition[] initialPositions;

  private double forwardError, sidewaysError = 0;


  /**
   * Creates a new AutonomousDistanceDriveCommand.
   *
   * @param drivetrain The instance of the Drivetrain subsystem declared in RobotContainer.
   * @param speeds The desired robot-relative forward and sideways velocities in meters/second for the robot 
   *               to move at.
   * @param distances The desired robot-relative forward and sideways distances in meters for the robot to travel.
   */
  public AutonomousDistanceDriveCommand(Drivetrain drivetrain, Translation2d speeds, Translation2d distances) {
    m_drivetrain = drivetrain;

    forwardSpeed = speeds.getX();
    sidewaysSpeed = speeds.getY();

    targetForwardDistance = Math.abs(distances.getX()) * Math.signum(forwardSpeed);
    targetSidewaysDistance = Math.abs(distances.getY()) * Math.signum(sidewaysSpeed);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initialPositions = m_drivetrain.getPositions();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forwardDistance = 0;
    double sidewaysDistance = 0;

    SwerveModulePosition[] currentPositions = m_drivetrain.getPositions();

    for(int i = 0; i < 4; i ++) {

      forwardDistance += (currentPositions[i].angle.getCos() * currentPositions[i].distanceMeters) - (initialPositions[i].angle.getCos() * initialPositions[i].distanceMeters);

    }

    for(int i = 0; i < 4; i ++) {

      sidewaysDistance += (currentPositions[i].angle.getSin() * currentPositions[i].distanceMeters) - (initialPositions[i].angle.getSin() * initialPositions[i].distanceMeters);

    }

    forwardDistance /= 4;
    sidewaysDistance /= 4;

    forwardError = Math.abs(Math.abs(targetForwardDistance) - Math.abs(forwardDistance));
    sidewaysError = Math.abs(Math.abs(targetSidewaysDistance) - Math.abs(sidewaysDistance));

    System.out.println(forwardError);
    System.out.println(sidewaysError);

    if(forwardError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE) {

      forwardSpeed = 0;

    }
    if(sidewaysError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE) {

      sidewaysSpeed = 0;

    }

    // Sets the x speed and y speed of the robot
    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(forwardSpeed, sidewaysSpeed), new Rotation2d()), false, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(), new Rotation2d()), false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return 
      (forwardError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE) && 
      (sidewaysError < Constants.kDrivetrain.AUTO_DISTANCE_ERROR_TOLERANCE);

  }
}