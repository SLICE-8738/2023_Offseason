// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
//import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
//import frc.robot.commands.InstantCalibrationCommand;
//import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
//import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneHighRowMobilityAndEngageMode extends SequentialCommandGroup {
  /** Creates a new SocreOneCubeGoOutThenEngageMode. */
  public ScoreOneHighRowMobilityAndEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive/*, Elevator elevator, Wrist wrist, Intake intake*/) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    //PlaceHighRowSequence placeCube = new PlaceHighRowSequence(elevator, wrist, intake);
    GridOutOfCommunityToChargeStationPath gridOutOfCommunityToChargeStation = new GridOutOfCommunityToChargeStationPath(startPosition);
    BoardChargeStationCommand getOnChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    Field2dTrajectoryFollowerSequence trajectory = new Field2dTrajectoryFollowerSequence(drive, gridOutOfCommunityToChargeStation, gridOutOfCommunityToChargeStation.getPathStartingPosition());

    addCommands(
      //calibrateElevatorAndWrist,
      //placeCube,
      //driveBack,
      //quickTurn,
      trajectory,
      getOnChargeStation,
      chargeStationBalance
    );

  }

}
