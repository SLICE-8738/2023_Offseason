// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.ChargeStation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ChargeStationBalancePIDCommand extends Command {

  private final Drivetrain m_drivetrain;

  private double pitch;

  private final PIDController pidController;

  /** Creates a new ChargeStationBalanceCommand. */
  public ChargeStationBalancePIDCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    //pidController = new PIDController(0.032, 0.0001, 0.0009);
    pidController = new PIDController(0.032, 0.0001, 0.0009);
    pidController.setSetpoint(0);

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

    //if (Math.abs(pitch) > 2) {
    if (Math.abs(pitch) > 0.5) {  
      m_drivetrain.swerveDrive(new Transform2d(new Translation2d(pidController.calculate(pitch), 0), new Rotation2d()), false, false);
    } else {
      m_drivetrain.swerveDrive(new Transform2d(new Translation2d(), new Rotation2d()), false, false);
      pidController.reset();
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
