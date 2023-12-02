// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Constants;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.paths.GamePieceToGridPath;
//import frc.robot.commands.GoToState;
import frc.robot.commands.Drivetrain.AutoDrive.AutonomousDistanceDriveCommand;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalanceCommand;
import frc.robot.commands.Drivetrain.sequences.TrajectoryFollowerSequence;
//import frc.robot.commands.StateSequences.IntakeCommandsSequence;
//import frc.robot.commands.StateSequences.OuttakeAndStowCommandsSequence;
//import frc.robot.commands.StowCommands.Stow;
//import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Arm.StowState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoHighAndMidRowAndEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreTwoGamePiecesThenEngageMode. */
  public ScoreTwoHighAndMidRowAndEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, /*
      Elevator elevator, Arm arm, Intake intake,*/ BooleanSupplier onBlueAlliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //GoToState toMid = new GoToState(elevator, arm, Constants.kRobotStates.midScore);
    //OuttakeAndStowCommandsSequence scoreHighAndStow = new OuttakeAndStowCommandsSequence(intake, arm, elevator, Constants.kRobotStates.highScore);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition);

    //IntakeCommandsSequence groundConeIntake = new IntakeCommandsSequence(intake, arm, elevator, StowState.Cone, Constants.kRobotStates.uprightConeGround);
    AutonomousDistanceDriveCommand driveForward = new AutonomousDistanceDriveCommand(drive, new Translation2d(1, 0), new Translation2d(0.25, 0));
    //Stow stow = new Stow(elevator, arm, intake);
    //SequentialCommandGroup pickUpGamePiece = new SequentialCommandGroup(new ParallelRaceGroup(groundConeIntake, driveForward.beforeStarting(new WaitCommand(1))), stow);

    GamePieceToGridPath gamePieceToGrid = new GamePieceToGridPath(startPosition);
    //OuttakeAndStowCommandsSequence scoreMidAndStow = new OuttakeAndStowCommandsSequence(intake, arm, elevator, Constants.kRobotStates.midScore);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition);
    BoardChargeStationCommand boardChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalanceCommand balance = new ChargeStationBalanceCommand(drive);

    TrajectoryFollowerSequence trajectory1 = new TrajectoryFollowerSequence(drive, gridToGamePiece);
    TrajectoryFollowerSequence trajectory2 = new TrajectoryFollowerSequence(drive, gamePieceToGrid);
    TrajectoryFollowerSequence trajectory3 = new TrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      //toMid,
      //scoreHighAndStow,
      trajectory1
      //pickUpGamePiece
      // trajectory2,
      // scoreMidAndStow,
      // trajectory3,
      // boardChargeStation,
      // balance
    );

  }

}
