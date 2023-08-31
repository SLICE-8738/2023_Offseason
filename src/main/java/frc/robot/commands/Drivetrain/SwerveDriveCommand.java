// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.JoystickFilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  private final Drivetrain m_drivetrain;
  private final Elevator m_elevator;

  private final GenericHID m_driverController;
  private final JoystickFilter translationXFilter, translationYFilter, rotationFilter;

  private final boolean m_isOpenLoop;
  private final boolean m_isFieldRelative;

  public SwerveDriveCommand(Drivetrain drivetrain, GenericHID driverController, boolean isOpenLoop, boolean isFieldRelative, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_elevator = elevator;

    m_driverController = driverController;

    m_isOpenLoop = isOpenLoop;
    m_isFieldRelative = isFieldRelative;

    translationXFilter = new JoystickFilter(0.07, 0.9);
    translationYFilter = new JoystickFilter(0.07, 0.9);
    rotationFilter = new JoystickFilter(0.07, 0.3);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Slow down the robot when the elevator is up
    double elevatorPercentage = m_elevator.getElevatorHeight() / Constants.kElevator.MAX_ELEVATOR_HEIGHT;
    double drivePercent = elevatorPercentage < 0.5 ? 1 : (1 - elevatorPercentage) * 1.5 + 0.25;

    drivePercent = 0.10;
    
    double translationX = translationXFilter.filter(m_driverController.getRawAxis(1)) * Constants.kDrivetrain.MAX_LINEAR_VELOCITY * drivePercent;
    double translationY = -translationYFilter.filter(m_driverController.getRawAxis(0)) * Constants.kDrivetrain.MAX_LINEAR_VELOCITY * drivePercent;
    double rotation = rotationFilter.filter(m_driverController.getRawAxis(2)) * Constants.kDrivetrain.MAX_ANGULAR_VELOCITY * drivePercent;

    m_drivetrain.swerveDrive(
      new Transform2d(new Translation2d(translationX, translationY), new Rotation2d(rotation)),
      m_isOpenLoop,
      m_isFieldRelative);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(new Transform2d(new Translation2d(0, 0), new Rotation2d()), m_isOpenLoop, m_isFieldRelative);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}