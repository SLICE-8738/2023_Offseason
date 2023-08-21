// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothingCommand;
//import frc.robot.commands.DoNothingCommand;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Wrist;
//import frc.robot.commands.GoToStateCommand;
//import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
/*import frc.robot.commands.sequences.PlaceConeMidRowSequence;
import frc.robot.commands.sequences.PlaceCubeMidRowSequence;
import frc.robot.commands.sequences.PlaceGamePieceLowRowSequence;
import frc.robot.commands.sequences.PlaceHighRowSequence;*/
//import frc.robot.commands.sequences.ToHighRowSequence;

/**
 * This class creates automatic node alignment and positioning sequences using the node selector
 * in Shuffleboard.
 */
public class NodeSelector {

    public enum Alliance {

        BLUE,
        RED

    }

    private final ShuffleboardTab driverTab;
    private final ShuffleboardLayout nodeSelectorLayout;
  
    private final ArrayList<GenericEntry> nodeButtons;
    private final SendableChooser<Alliance> allianceSelector;

    private GenericEntry storedSelectedButton;
    private Alliance storedSelectedAlliance;

    private static SequentialCommandGroup selectedSequence;

    private static Pose2d nodePosition;

    private final Drivetrain m_drivetrain;
    //private final Elevator m_elevator;
    //private final Wrist m_wrist;
    //private final Intake m_intake;

    public NodeSelector(Drivetrain drivetrain/*, Elevator elevator, Wrist wrist, Intake intake*/) {

        m_drivetrain = drivetrain;
        //m_elevator = elevator;
        //m_wrist = wrist;
        //m_intake = intake;
        
        driverTab = Shuffleboard.getTab("Driver Tab");

        nodeSelectorLayout = driverTab.getLayout("Node Selection", BuiltInLayouts.kGrid).
        withPosition(1, 2).
        withSize(7, 2).
        withProperties(Map.of("Number of columns", 9, "Number of rows", 3));
    
        nodeButtons = new ArrayList<GenericEntry>();
        allianceSelector = new SendableChooser<Alliance>();
    
        for(int i = 0; i < 9; i ++) {
    
          nodeButtons.add(nodeSelectorLayout.add("High Row " + (i + 1), false).
          withWidget(BuiltInWidgets.kToggleButton).
          withPosition(i, 0).
          getEntry());
    
        }
        for(int i = 0; i < 9; i ++) {
    
          nodeButtons.add(nodeSelectorLayout.add("Mid Row " + (i + 1), false).
          withWidget(BuiltInWidgets.kToggleButton).
          withPosition(i, 1).
          getEntry());
    
        }
        for(int i = 0; i < 9; i ++) {
    
          nodeButtons.add(nodeSelectorLayout.add("Low Row " + (i + 1), false).
          withWidget(BuiltInWidgets.kToggleButton).
          withPosition(i, 2).
          getEntry());
    
        }

        allianceSelector.setDefaultOption("Blue Alliance", Alliance.BLUE);
        allianceSelector.addOption("Red Alliance", Alliance.RED);

        driverTab.add("Alliance", allianceSelector).withPosition(3, 1).withSize(3, 1);

        nodeButtons.get(0).setBoolean(true);
        updateSequenceCreator();

    }

    public void updateNodePosition() {
 
        for(int i = 0; i < 27; i ++) {

            GenericEntry currentButton = nodeButtons.get(i);

            if(currentButton.getBoolean(false)) {

                if(currentButton != storedSelectedButton) {

                    System.out.println("Node selection changed, updating position");

                    if(storedSelectedButton != null) {

                        storedSelectedButton.setBoolean(false);

                    }

                    storedSelectedButton = currentButton;
                    nodePosition = getPositionForIndex(i);

                }

            }

        }

    }

    public void updateSequenceCreator() {

        for(int i = 0; i < 27; i ++) {

            GenericEntry currentButton = nodeButtons.get(i);
            Alliance selectedAlliance = allianceSelector.getSelected();

            if(currentButton.getBoolean(false)) {

                if(currentButton != storedSelectedButton) {

                    System.out.println("Node selection changed, updating sequence");

                    if(storedSelectedButton != null) {

                        storedSelectedButton.setBoolean(false);

                    }

                    storedSelectedButton = currentButton;
                    nodePosition = getPositionForIndex(i);
                    selectedSequence = getSequenceForIndex(i);

                }

            }
            if(selectedAlliance != storedSelectedAlliance) {

                System.out.println("Alliance selection changed, updating sequence");
                selectedSequence = getSequenceForIndex(nodeButtons.indexOf(storedSelectedButton));

            }

            storedSelectedAlliance = selectedAlliance;

        }

    }

    public Pose2d getPositionForIndex(int selectedNodeIndex) {

        boolean onBlueAlliance;

        if(allianceSelector.getSelected() == Alliance.BLUE) {

            onBlueAlliance = true;

        }
        else {

            onBlueAlliance = false;

        }

        if(selectedNodeIndex == 0 || selectedNodeIndex == 9 || selectedNodeIndex == 18) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 0.5:4.95, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 1 || selectedNodeIndex == 10 || selectedNodeIndex == 19) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 1.06:4.42, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 2 || selectedNodeIndex == 11 || selectedNodeIndex == 20) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 1.63:3.85, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 3 || selectedNodeIndex == 12 || selectedNodeIndex == 21) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 2.18:3.3, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 4 || selectedNodeIndex == 13 || selectedNodeIndex == 22) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, 2.74, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 5 || selectedNodeIndex == 14 || selectedNodeIndex == 23) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 3.3:2.18, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 6 || selectedNodeIndex == 15 || selectedNodeIndex == 24) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 3.85:1.63, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 7 || selectedNodeIndex == 16 || selectedNodeIndex == 25) {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 4.42:1.06, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else {
            return new Pose2d(onBlueAlliance? 1.81:14.72, onBlueAlliance? 4.95:0.5, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        }

    }

    public SequentialCommandGroup getSequenceForIndex(int selectedNodeIndex) {

        //Pose2d initialPosition = Limelight.getLastBotPoseBlue();
        Pose2d initialPosition = m_drivetrain.getPose();
        Translation2d interiorWaypoint;
        Pose2d finalPosition;

        //Command positionSequence;

        boolean onBlueAlliance;

        if(allianceSelector.getSelected() == Alliance.BLUE) {

            onBlueAlliance = true;

        }
        else {

            onBlueAlliance = false;

        }

        /*if(selectedNodeIndex < 9) {
            positionSequence = new ToHighRowSequence(m_elevator, m_wrist);
        } else if(selectedNodeIndex == 9 || selectedNodeIndex == 11 || selectedNodeIndex == 12 || selectedNodeIndex == 14 || selectedNodeIndex == 15 || selectedNodeIndex == 17) {
            positionSequence = new GoToStateCommand(m_elevator, m_wrist, Constants.States.MID_ROW_CONE_STATE);
        } else if(selectedNodeIndex == 10 || selectedNodeIndex == 13 || selectedNodeIndex == 16) {
            positionSequence = new GoToStateCommand(m_elevator, m_wrist, Constants.States.MID_ROW_CUBE_STATE);
        } else {
            positionSequence = new GoToStateCommand(m_elevator, m_wrist, Constants.States.LOW_ROW_GROUND_STATE);
        }*/

        finalPosition = nodePosition;

        if(
            (selectedNodeIndex == 0 || selectedNodeIndex == 1 || selectedNodeIndex == 7 || selectedNodeIndex == 8 ||
            selectedNodeIndex == 9 || selectedNodeIndex == 10 || selectedNodeIndex == 16 || selectedNodeIndex == 17 ||
            selectedNodeIndex == 18 || selectedNodeIndex == 19 || selectedNodeIndex == 25 || selectedNodeIndex == 26) ||
            (initialPosition.getX() < 2.75 || initialPosition.getX() > 13.81)
            ) {
            interiorWaypoint = new Translation2d((initialPosition.getX() + finalPosition.getX()) / 2, (initialPosition.getY() + finalPosition.getY()) / 2);
        } else  {
            interiorWaypoint = new Translation2d(onBlueAlliance? 2.65:13.91 , initialPosition.getY());
        }

        SmartDashboard.putNumber("Initial Position X", initialPosition.getX());
        SmartDashboard.putNumber("Interior Waypoint X", interiorWaypoint.getX());
        SmartDashboard.putNumber("Final Position X", finalPosition.getX());

        try {

        return new SequentialCommandGroup(
                        new Field2dTrajectoryFollowerSequence(
                            m_drivetrain, 
                            TrajectoryGenerator.generateTrajectory(
                                initialPosition, 
                                List.of(interiorWaypoint), 
                                finalPosition, 
                                new TrajectoryConfig(Constants.kAutonomous.kMaxVelocityMetersPerSecond, Constants.kAutonomous.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDrivetrain.kSwerveKinematics)))
                        /*positionSequence*/);
                    
        }
        catch (TrajectoryGenerationException e) {

            return new SequentialCommandGroup(new DoNothingCommand());

        }


    }

    public SequentialCommandGroup getNodeSequence() {

        return selectedSequence;

    }

    public static Pose2d getNodePosition() {

        return nodePosition;

    }

}