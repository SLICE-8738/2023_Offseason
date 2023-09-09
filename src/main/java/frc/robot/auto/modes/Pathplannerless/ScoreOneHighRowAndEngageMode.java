// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplannerless;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.commands.GoToState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalanceCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneHighRowAndEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneConeHighRowMode. */
  public ScoreOneHighRowAndEngageMode(Drivetrain drivetrain, Elevator elevator, Arm arm, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //SetInitialPositionCommand setInitialPosition = new SetInitialPositionCommand(drivetrain);
    //InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    //PlaceHighRowSequence placeCone = new PlaceHighRowSequence(elevator, wrist, intake);
    /*GoToState toMid = new GoToState(elevator, arm, Constants.kRobotStates.midScore);
    GoToState toHigh = new GoToState(elevator, arm, Constants.kRobotStates.highScore);
    ParallelRaceGroup outtake = new OutTakeCommand(intake).withTimeout(1);
    GoToState stow = new GoToState(elevator, arm, Constants.kRobotStates.stow);*/

    BoardChargeStationCommand boardChargeStation = new BoardChargeStationCommand(drivetrain);
    ChargeStationBalanceCommand balance = new ChargeStationBalanceCommand(drivetrain);

    addCommands(
      /*toMid,
      toHigh,
      outtake,
      stow,*/
      boardChargeStation,
      balance
    );

  }

}
