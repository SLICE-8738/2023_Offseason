// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//import frc.robot.Constants;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GamePieceToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
//import frc.robot.commands.StateSequences.IntakeAndStowCommandsSequence;
//import frc.robot.commands.StateSequences.OuttakeAndStowCommandsSequence;
//import frc.robot.commands.GoToState;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalanceCommand;
//import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Arm.StowState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneHighRowPickUpAndEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneCubePickUpOneGamePieceThenEngageMode. */
  public ScoreOneHighRowPickUpAndEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive/*, Elevator elevator, Arm arm, Intake intake*/) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //GoToState toMid = new GoToState(elevator, arm, Constants.kRobotStates.midScore);
    //OuttakeAndStowCommandsSequence scoreHighAndStow = new OuttakeAndStowCommandsSequence(intake, arm, elevator, Constants.kRobotStates.highScore);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition);
    //IntakeAndStowCommandsSequence intakeCone = new IntakeAndStowCommandsSequence(intake, arm, elevator, StowState.Cone, Constants.kRobotStates.uprightConeGround);
    GamePieceToChargeStationPath gamePieceToChargeStation = new GamePieceToChargeStationPath(startPosition);
    BoardChargeStationCommand boardChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalanceCommand balance = new ChargeStationBalanceCommand(drive);

    Field2dTrajectoryFollowerSequence trajectory1 = new Field2dTrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.getTrajectory().getInitialTargetHolonomicPose());
    Field2dTrajectoryFollowerSequence trajectory2 = new Field2dTrajectoryFollowerSequence(drive, gamePieceToChargeStation);

    addCommands(
      //toMid,
      //scoreHighAndStow,
      trajectory1,
      //intakeCone,
      trajectory2,
      boardChargeStation,
      balance
    );

  }

}
