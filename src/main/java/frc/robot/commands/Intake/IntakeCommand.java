// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.StowState;

public class IntakeCommand extends CommandBase {
  private Intake intake;
  private final Timer timer;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Will initialize when go to position is done
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Checks to see if it's a cone or cube being picked up and spins in the
    // appropriate direction.

    if (Arm.stowState == StowState.Cube) {
      intake.IntakeSpinUpCube();
      // arm.wristSecureCube();
    }

    else if (Arm.stowState == StowState.Cone) {
      intake.IntakeSpinOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.IntakeStopSpinning();
    intake.setGamePieceSecured(true);
    // Ends when Stow is called by operator //

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (timer.get() <= 1.5) {
      return false;
    }

    return intake.getOutputCurrent() > Constants.kIntake.CONE_THRESHOLD;

  }
}
