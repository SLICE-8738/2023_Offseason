// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.modes.Pathplanner.ScoreOneHighRowMobilityAndEngageMode;
import frc.robot.auto.modes.Pathplanner.ScoreOneHighRowPickUpAndEngageMode;
import frc.robot.auto.modes.Pathplanner.ScoreTwoHighAndMidRowMode;
import frc.robot.auto.modes.Pathplanner.ScoreTwoHighAndMidRowAndEngageMode;
import frc.robot.auto.modes.Pathplannerless.ScoreOneHighRowMode;
import frc.robot.auto.modes.Pathplannerless.ScoreOneHighRowAndMobilityMode;
import frc.robot.auto.modes.Pathplannerless.ScoreOneHighRowAndEngageMode;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

import java.util.Optional;

/**
 * This class primarily manages the creation and updating of the autonomous mode
 * and starting position sendable choosers on Shuffleboard.
 * 
 * <p> {@link SendableChooser See SendableChooser class here}
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
        
        SCORE_ONE_HIGH_ROW_PATHPLANNERLESS,
        SCORE_ONE_HIGH_ROW_AND_MOBILITY_PATHPLANNERLESS,
        SCORE_ONE_HIGH_ROW_AND_ENGAGE_PATHPLANNERLESS,
        SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE_PATHPLANNER,
        SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE_PATHPLANNER,
        SCORE_TWO_HIGH_AND_MID_ROW_PATHPLANNER,
        SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE_PATHPLANNER,

    }

    public static StartingPosition storedStartingPosition;
    public DesiredMode storedDesiredMode;

    public SendableChooser<StartingPosition> startingPositionChooser;
    public SendableChooser<DesiredMode> modeChooser;

    private Optional<SequentialCommandGroup> autoMode = Optional.empty();

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final Intake m_intake;

    public AutoSelector(Drivetrain drivetrain, Elevator elevator, Arm arm, Intake intake) {

        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_arm = arm;
        m_intake = intake;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Blue Left", StartingPosition.BLUE_COMMUNITY_LEFT);

        startingPositionChooser.addOption("Blue Middle", StartingPosition.BLUE_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Blue Right", StartingPosition.BLUE_COMMUNITY_RIGHT);
        startingPositionChooser.addOption("Red Left", StartingPosition.RED_COMMUNITY_LEFT);
        startingPositionChooser.addOption("Red Middle", StartingPosition.RED_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Red Right", StartingPosition.RED_COMMUNITY_RIGHT);

        modeChooser = new SendableChooser<DesiredMode>();

        modeChooser.setDefaultOption("Any - Score One High Row ", DesiredMode.SCORE_ONE_HIGH_ROW_PATHPLANNERLESS);

        modeChooser.addOption("Any - Score One High Row And Mobility ", DesiredMode.SCORE_ONE_HIGH_ROW_AND_MOBILITY_PATHPLANNERLESS);
        modeChooser.addOption("Middle - Score One High Row And Engage ", DesiredMode.SCORE_ONE_HIGH_ROW_AND_ENGAGE_PATHPLANNERLESS);

        modeChooser.addOption("(Pathplanner) Score One High Row Mobility And Engage", DesiredMode.SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE_PATHPLANNER);
        modeChooser.addOption("(Pathplanner) Score One High Row Pick Up Piece And Engage", DesiredMode.SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE_PATHPLANNER);
        modeChooser.addOption("(Pathplanner) Score Two High and Mid Row", DesiredMode.SCORE_TWO_HIGH_AND_MID_ROW_PATHPLANNER);
        modeChooser.addOption("(Pathplanner) Score Two High and Mid Row And Engage", DesiredMode.SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE_PATHPLANNER);

    }

    public void updateAutoSelector() {

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if(storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {
            
            System.out.println("Auto selection changed, updating creator; Starting Position: " + startingPosition.name()
            + ", Desired Mode: " + desiredMode.name());

            autoMode = getAutoModeForParams(startingPosition, desiredMode);

            updateInitialAutoPoseOffset(startingPosition, desiredMode);

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<SequentialCommandGroup> getAutoModeForParams(StartingPosition position, DesiredMode mode) {

        switch(mode) {

            case SCORE_ONE_HIGH_ROW_PATHPLANNERLESS:
                return Optional.of(new ScoreOneHighRowMode(m_drivetrain, m_elevator, m_arm, m_intake));
            case SCORE_ONE_HIGH_ROW_AND_MOBILITY_PATHPLANNERLESS:
                return Optional.of(new ScoreOneHighRowAndMobilityMode(m_drivetrain, m_elevator, m_arm, m_intake, () -> storedStartingPosition.name().startsWith("BLUE")));
            case SCORE_ONE_HIGH_ROW_AND_ENGAGE_PATHPLANNERLESS:
                return Optional.of(new ScoreOneHighRowAndEngageMode(m_drivetrain, m_elevator, m_arm, m_intake));
            case SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE_PATHPLANNER:
                return Optional.of(new ScoreOneHighRowMobilityAndEngageMode(position, m_drivetrain, m_elevator, m_arm, m_intake));
            case SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE_PATHPLANNER:
                return Optional.of(new ScoreOneHighRowPickUpAndEngageMode(position, m_drivetrain, m_elevator, m_arm, m_intake));
            case SCORE_TWO_HIGH_AND_MID_ROW_PATHPLANNER:
                return Optional.of(new ScoreTwoHighAndMidRowMode(position, m_drivetrain, m_elevator, m_arm, m_intake));
            case SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE_PATHPLANNER:
                return Optional.of(new ScoreTwoHighAndMidRowAndEngageMode(position, m_drivetrain, m_elevator, m_arm, m_intake));
            default:
                break;
    
        }

        System.err.println("No valid auto mode found for " + mode);
        return Optional.empty();

    }

    public void updateInitialAutoPoseOffset(StartingPosition startingPosition, DesiredMode desiredMode) {

        /*This variable should be should be assigned to Limelight.getLastBotPoseBlue() instead when
        the Limelight subsystem file is added*/
        Pose2d botPose = m_drivetrain.getPose();

        if(desiredMode.name().endsWith("PATHPLANNER")) {

            switch(desiredMode) {

                case SCORE_ONE_HIGH_ROW_MOBILITY_AND_ENGAGE_PATHPLANNER:
                    initialAutoPose = new GridOutOfCommunityToChargeStationPath(startingPosition).getPathInitialState().poseMeters;
                    break;
                case SCORE_ONE_HIGH_ROW_PICK_UP_AND_ENGAGE_PATHPLANNER:
                    initialAutoPose = new GridToGamePiecePath(startingPosition).getPathInitialState().poseMeters;
                    break;
                case SCORE_TWO_HIGH_AND_MID_ROW_PATHPLANNER:
                    initialAutoPose = new GridToGamePiecePath(startingPosition).getPathInitialState().poseMeters;
                    break;
                case SCORE_TWO_HIGH_AND_MID_ROW_AND_ENGAGE_PATHPLANNER:
                    initialAutoPose = new GridToGamePiecePath(startingPosition).getPathInitialState().poseMeters;
                    break;
                default:
                    System.err.println("No valid initial auto pose found for " + desiredMode);
                    break;
                
            }

        }
        else {

            System.out.println("No initial pose is available for Pathplannerless modes");
            initialAutoPose = botPose;

        }

        if(botPose != null && initialAutoPose != null) {

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

    public SequentialCommandGroup getAutoMode() {

        return autoMode.get();

    }

    public String getStoredDesiredMode() {

        if(storedDesiredMode != null) {

            return storedDesiredMode.name();

        }
        else {

            return "None Stored";

        }

    }

    public static StartingPosition getStoredStartingPosition() {

        return storedStartingPosition;

    }

    public String getStoredStartingPositionName() {

        if(storedStartingPosition != null) {

            return storedStartingPosition.name();

        }
        else {

            return "None Stored";

        }

    }

}
