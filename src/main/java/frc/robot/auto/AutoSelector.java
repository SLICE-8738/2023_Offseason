// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants;
import frc.robot.commands.GoToState;
import frc.robot.commands.Drivetrain.AutoDrive.AutonomousDistanceDriveCommand;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalanceCommand;
import frc.robot.commands.StateSequences.IntakeAndStowCommandsSequence;
import frc.robot.commands.StateSequences.OuttakeAndStowCommandsSequence;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.StowState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class primarily manages the creation and updating of the autonomous mode
 * and starting position sendable choosers on Shuffleboard.
 * 
 * <p>
 * {@link SendableChooser See SendableChooser class here}
 */
public class AutoSelector {

    public enum StartingPosition {

        BLUE_COMMUNITY_LEFT,
        BLUE_COMMUNITY_MIDDLE,
        BLUE_COMMUNITY_RIGHT,
        RED_COMMUNITY_LEFT,
        RED_COMMUNITY_MIDDLE,
        RED_COMMUNITY_RIGHT

    }

    public enum DesiredMode {

        SCORE_ONE_HIGH_ROW,
        SCORE_ONE_HIGH_ROW_AND_MOBILITY,
        SCORE_ONE_HIGH_ROW_AND_ENGAGE,
        SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE,
        SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE,
        SCORE_TWO_HIGH_AND_MID_ROW,
        SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE,
        BACKWARD_TEST_TRAJECTORY_MODE,
        BACKWARD_AND_LEFT_TEST_TRAJECTORY_MODE,
        FORWARD_AND_RIGHT_TEST_TRAJECTORY_MODE

    }

    public static StartingPosition storedStartingPosition;
    public DesiredMode storedDesiredMode;

    public SendableChooser<StartingPosition> startingPositionChooser;
    public SendableChooser<DesiredMode> modeChooser;

    private Optional<PathPlannerAuto> autoMode = Optional.empty();
    private String autoName;

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;
    private final Arm m_arm;
    private final Elevator m_elevator;
    private final Intake m_intake;

    public AutoSelector(Drivetrain drivetrain, Arm arm, Elevator elevator, Intake intake) {

        m_drivetrain = drivetrain;
        m_arm = arm;
        m_elevator = elevator;
        m_intake = intake;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Blue Left", StartingPosition.BLUE_COMMUNITY_LEFT);

        startingPositionChooser.addOption("Blue Middle", StartingPosition.BLUE_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Blue Right", StartingPosition.BLUE_COMMUNITY_RIGHT);
        startingPositionChooser.addOption("Red Left", StartingPosition.RED_COMMUNITY_LEFT);
        startingPositionChooser.addOption("Red Middle", StartingPosition.RED_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Red Right", StartingPosition.RED_COMMUNITY_RIGHT);

        modeChooser = new SendableChooser<DesiredMode>();

        modeChooser.setDefaultOption("Any - Score One High Row ", DesiredMode.SCORE_ONE_HIGH_ROW);

        modeChooser.addOption("Score One High Row And Mobility ",
                DesiredMode.SCORE_ONE_HIGH_ROW_AND_MOBILITY);
        modeChooser.addOption("Score One High Row And Engage ",
                DesiredMode.SCORE_ONE_HIGH_ROW_AND_ENGAGE);

        modeChooser.addOption("Score One High Row Mobility And Engage",
                DesiredMode.SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE);
        modeChooser.addOption("Score One High Row Pick Up Piece And Engage",
                DesiredMode.SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE);
        modeChooser.addOption("Score Two High and Mid Row",
                DesiredMode.SCORE_TWO_HIGH_AND_MID_ROW);
        modeChooser.addOption("Score Two High and Mid Row And Engage",
                DesiredMode.SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE);
        modeChooser.addOption("Backward Test Trajectory Mode", 
                DesiredMode.BACKWARD_TEST_TRAJECTORY_MODE);
        modeChooser.addOption("Backward Test Trajectory Mode", 
                DesiredMode.BACKWARD_AND_LEFT_TEST_TRAJECTORY_MODE);
        modeChooser.addOption("Backward Test Trajectory Mode", 
                DesiredMode.FORWARD_AND_RIGHT_TEST_TRAJECTORY_MODE);

        AutoBuilder.configureHolonomic(
            m_drivetrain::getPose,
            m_drivetrain::resetOdometry,
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.kAutonomous.kPTranslation),
                new PIDConstants(Constants.kAutonomous.kPRotation),
                Constants.kAutonomous.kMaxVelocityMetersPerSecond,
                Constants.kDrivetrain.DRIVE_BASE_RADIUS,
                new ReplanningConfig(false, false)),
            m_drivetrain);

        NamedCommands.registerCommand("To Mid", new GoToState(elevator, arm, Constants.kRobotStates.midScore));
        NamedCommands.registerCommand("Score High And Stow",  new OuttakeAndStowCommandsSequence(intake, arm, elevator, Constants.kRobotStates.highScore));
        NamedCommands.registerCommand("Score Mid And Stow", new OuttakeAndStowCommandsSequence(intake, arm, elevator, Constants.kRobotStates.midScore));
        NamedCommands.registerCommand("Intake Cone And Stow", new IntakeAndStowCommandsSequence(intake, arm, elevator, StowState.Cone, Constants.kRobotStates.uprightConeGround));
        NamedCommands.registerCommand("Mobility", new AutonomousDistanceDriveCommand(drivetrain, new Translation2d(2, 0), new Translation2d(6, 0)));
        NamedCommands.registerCommand("Board Charge Station", new BoardChargeStationCommand(drivetrain));
        NamedCommands.registerCommand("Balance", new ChargeStationBalanceCommand(drivetrain));

    }

    public void updateAutoSelector() {

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if (storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + startingPosition.name()
                    + ", Desired Mode: " + desiredMode.name());

            autoMode = getAutoModeForParams(startingPosition, desiredMode);

            updateInitialAutoPoseOffset();

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<PathPlannerAuto> getAutoModeForParams(StartingPosition position, DesiredMode mode) {

        switch (mode) {

            case SCORE_ONE_HIGH_ROW:
                autoName = "Score One High Row";
                break;
            case SCORE_ONE_HIGH_ROW_AND_MOBILITY:
                autoName = "Score One High Row And Mobility";
                break;
            case SCORE_ONE_HIGH_ROW_AND_ENGAGE:
                autoName = "Score One High Row And Engage";
                break;
            case SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE:
                autoName = "Score One High Row Mobility And Engage";
                break;
            case SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE:
                autoName = "Score One High Row Pick Up And Engage";
                break;
            case SCORE_TWO_HIGH_AND_MID_ROW:
                autoName = "Score Two High And Mid Row";
                break;
            case SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE:
                autoName = "Score Two High And Mid Row And Engage";
                break;
            case BACKWARD_TEST_TRAJECTORY_MODE:
                autoName = "Backward Test Auto";
                break;
            case BACKWARD_AND_LEFT_TEST_TRAJECTORY_MODE:
                autoName = "Backward and Left Test Auto";
                break;
            case FORWARD_AND_RIGHT_TEST_TRAJECTORY_MODE:
                autoName = "Forward and Right Test Auto";
                break;
            default:
                System.err.println("No valid auto mode found for " + mode);
                return Optional.empty();

        }

        return Optional.of(new PathPlannerAuto(autoName));

    }

    public void updateInitialAutoPoseOffset() {

        Pose2d botPose = m_drivetrain.getPose();

        initialAutoPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        if (botPose != null && initialAutoPose != null) {

            initialAutoPoseXOffset = Math.abs(initialAutoPose.getX() - botPose.getX());
            initialAutoPoseYOffset = Math.abs(initialAutoPose.getY() - botPose.getY());
            initialAutoPoseRotationOffset = initialAutoPose.getRotation().getDegrees() - botPose.getRotation().getDegrees();

        }

    }

    public void reset() {

        autoMode = Optional.empty();
        storedDesiredMode = null;

        initialAutoPose = null;

    }

    public PathPlannerAuto getAutoMode() {

        return autoMode.get();

    }

    public String getStoredDesiredMode() {

        if (storedDesiredMode != null) {

            return storedDesiredMode.name();

        } else {

            return "None Stored";

        }

    }

    public static StartingPosition getStoredStartingPosition() {

        return storedStartingPosition;

    }

    public String getStoredStartingPositionName() {

        if (storedStartingPosition != null) {

            return storedStartingPosition.name();

        } else {

            return "None Stored";

        }

    }

}