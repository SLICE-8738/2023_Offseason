// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.ChargeStation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ChargeStationBalanceCommand extends CommandBase {

  private final Drivetrain m_drivetrain;

  private double pitch;

  /** Creates a new ChargeStationBalanceCommand. */
  public ChargeStationBalanceCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /**
     * THIS SHOULD BE CHANGED TO getPitch()
     * IF THE PITCH AXIS IS PARALLEL TO THE ROBOT POINTING FORWARD 
     */
     pitch = m_drivetrain.getRoll();

    if(pitch > 10) {

      m_drivetrain.swerveDrive(new Transform2d(new Translation2d(-Constants.kDrivetrain.CHARGE_STATION_BALANCE_SPEED, 0), new Rotation2d()), true, false);

    }

    if(pitch < -10) {

      m_drivetrain.swerveDrive(new Transform2d(new Translation2d(Constants.kDrivetrain.CHARGE_STATION_BALANCE_SPEED, 0), new Rotation2d()), true, false);

    }

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
