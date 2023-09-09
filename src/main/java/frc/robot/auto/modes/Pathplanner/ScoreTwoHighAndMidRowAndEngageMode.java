// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.paths.GamePieceToGridPath;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
/*import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.sequences.PickUpGamePieceGroundSequence;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.commands.sequences.PlaceCubeMidRowSequence;*/
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoHighAndMidRowAndEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreTwoGamePiecesThenEngageMode. */
  public ScoreTwoHighAndMidRowAndEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive/*, Elevator elevator, Wrist wrist, Intake intake*/) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    //PlaceHighRowSequence placeCube1 = new PlaceHighRowSequence(elevator, wrist, intake);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition);
    //PickUpGamePieceGroundSequence pickUpGamePiece = new PickUpGamePieceGroundSequence(elevator, wrist, intake);
    GamePieceToGridPath gamePieceToGrid = new GamePieceToGridPath(startPosition);
    //PlaceCubeMidRowSequence placeCube2 = new PlaceCubeMidRowSequence(elevator, wrist, intake);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition);

    Field2dTrajectoryFollowerSequence trajectory1 = new Field2dTrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.getPathStartingPosition());
    Field2dTrajectoryFollowerSequence trajectory2 = new Field2dTrajectoryFollowerSequence(drive, gamePieceToGrid);
    Field2dTrajectoryFollowerSequence trajectory3 = new Field2dTrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      //calibrateElevatorAndWrist,
      //placeCube1,
      trajectory1,
      //pickUpGamePiece,
      trajectory2,
      //placeCube2,
      trajectory3
    );
    
  }
  
}
