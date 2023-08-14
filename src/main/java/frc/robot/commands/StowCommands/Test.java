// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StowCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.StowState;

public class Test extends CommandBase {
  /** Creates a new Test. */
  private Arm arm;
  Timer timer;
  
  public Test() {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    arm = new Arm();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getWristOutput()>Constants.kIntake.CONE_THRESHOLD) {
        Arm.stowState = StowState.Cone;
    } else if (arm.getArmOutput()>Constants.kArm.CUBE_THRESHOLD) {
        Arm.stowState = StowState.Cube;
    } else {
        Arm.stowState = StowState.Nothing;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get()>1);
  }
}
