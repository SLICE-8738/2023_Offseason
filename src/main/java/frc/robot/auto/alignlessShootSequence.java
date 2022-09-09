// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.AutoShoot.IndexerDownSlightCommand;
import frc.robot.commands.Indexer.AutoShoot.IndexerUpCommand;
import frc.robot.commands.Shooter.AutoShoot.ShooterFlywheelSpinDown;
import frc.robot.commands.Shooter.AutoShoot.ShooterFlywheelSpinUp;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignlessShootSequence extends SequentialCommandGroup {
  /** Creates a new alignlessShootSequence. */
  public alignlessShootSequence(Indexer indexer, Shooter shooter, Joystick leftJoystick, Joystick rightJoystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IndexerDownSlightCommand(indexer), new ShooterFlywheelSpinUp(shooter, leftJoystick, rightJoystick),
          new IndexerUpCommand(indexer), new ShooterFlywheelSpinDown(shooter));
  }
}
