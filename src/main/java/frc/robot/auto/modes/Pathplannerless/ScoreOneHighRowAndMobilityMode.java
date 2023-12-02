// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplannerless;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//import frc.robot.Constants;
//import frc.robot.commands.GoToState;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Arm;
import frc.robot.commands.Drivetrain.SetInitialPositionCommand;
import frc.robot.commands.Drivetrain.AutoDrive.AutonomousDistanceDriveCommand;
//import frc.robot.commands.StateSequences.OuttakeAndStowCommandsSequence;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneHighRowAndMobilityMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneConeHighRowMode. */
  public ScoreOneHighRowAndMobilityMode(Drivetrain drivetrain/*, Elevator elevator, Arm arm, Intake intake,*/) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    SetInitialPositionCommand setInitialPosition = new SetInitialPositionCommand(drivetrain);
    //GoToState toMid = new GoToState(elevator, arm, Constants.kRobotStates.midScore);
    //OuttakeAndStowCommandsSequence scoreHighAndStow = new OuttakeAndStowCommandsSequence(intake, arm, elevator, Constants.kRobotStates.highScore);
    AutonomousDistanceDriveCommand mobility = new AutonomousDistanceDriveCommand(drivetrain, new Translation2d(2, 0), new Translation2d(6, 0));

    addCommands(
        setInitialPosition,
        //toMid,
        //scoreHighAndStow,
        mobility);

  }

}
