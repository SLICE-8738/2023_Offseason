// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.AutonomousDistanceDriveCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.ChargeStationBalancePIDCommand;
import frc.robot.commands.Elevator.CalibrateElevatorCommand;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOnePieceThenMobility extends SequentialCommandGroup {
  /** Creates a new OneGamePieceGoOutThenEngageMode. */
  public ScoreOnePieceThenMobility(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);
    
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    
    //GridToGamePiecePath mobilityPath = new GridToGamePiecePath(startPosition);
    //Field2dTrajectoryFollowerSequence trajectory1 = new Field2dTrajectoryFollowerSequence(drive, mobilityPath);

    AutonomousDistanceDriveCommand mobility = new AutonomousDistanceDriveCommand(drive, 0.5, 3);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      //trajectory1
      mobility
    );

  }

}
