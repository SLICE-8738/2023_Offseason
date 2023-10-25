// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//import frc.robot.commands.GoToState;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
//import frc.robot.commands.StateSequences.ScoreHighSequence;
//import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;

/**
 * This class creates automatic node alignment and positioning sequences using the node selector
 * in Shuffleboard.
 */
public class NodeSelector {

    private final ShuffleboardTab driverTab;
    private final ShuffleboardLayout nodeSelectorLayout;
  
    private final ArrayList<GenericEntry> nodeButtons;
    private final SendableChooser<Alliance> allianceSelector;

    private GenericEntry storedSelectedButton;
    private Alliance storedSelectedAlliance;

    private static Pose2d nodePosition;

    private final Drivetrain m_drivetrain;
    //private final Elevator m_elevator;
    //private final Arm m_arm;

    public NodeSelector(Drivetrain drivetrain/*, Elevator elevator, Arm arm*/) {

        m_drivetrain = drivetrain;
        //m_elevator = elevator;
        //m_arm = arm;
        
        driverTab = Shuffleboard.getTab("Driver Tab");

        nodeSelectorLayout = driverTab.getLayout("Node Selection", BuiltInLayouts.kGrid).
        withPosition(2, 2).
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

        allianceSelector.setDefaultOption("Blue Alliance", Alliance.Blue);
        allianceSelector.addOption("Red Alliance", Alliance.Red);

        driverTab.add("Alliance", allianceSelector).withPosition(4, 1).withSize(3, 1);

        nodeButtons.get(0).setBoolean(true);

    }

    public void updateNodePosition() {

        for(int i = 0; i < 27; i ++) {

            GenericEntry currentButton = nodeButtons.get(i);
            Alliance selectedAlliance = allianceSelector.getSelected();

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
            if(selectedAlliance != storedSelectedAlliance) {

                System.out.println("Alliance selection changed, updating position");
                nodePosition = getPositionForIndex(i);

            }

            storedSelectedAlliance = selectedAlliance;

        }

    }

    public Pose2d getPositionForIndex(int selectedNodeIndex) {

        boolean onBlueAlliance = allianceSelector.getSelected() == Alliance.Blue;

        if(selectedNodeIndex == 0 || selectedNodeIndex == 9 || selectedNodeIndex == 18) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 0.5:4.95, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 1 || selectedNodeIndex == 10 || selectedNodeIndex == 19) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 1.06:4.42, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 2 || selectedNodeIndex == 11 || selectedNodeIndex == 20) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 1.63:3.85, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 3 || selectedNodeIndex == 12 || selectedNodeIndex == 21) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 2.18:3.3, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 4 || selectedNodeIndex == 13 || selectedNodeIndex == 22) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, 2.74, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 5 || selectedNodeIndex == 14 || selectedNodeIndex == 23) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 3.3:2.18, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 6 || selectedNodeIndex == 15 || selectedNodeIndex == 24) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 3.85:1.63, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else if(selectedNodeIndex == 7 || selectedNodeIndex == 16 || selectedNodeIndex == 25) {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 4.42:1.06, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        } else {
            return new Pose2d(onBlueAlliance? 1.92:14.61, onBlueAlliance? 4.95:0.5, Rotation2d.fromDegrees(onBlueAlliance? 180:0));
        }

    }

    public SequentialCommandGroup getNodeSequence() {

        boolean onBlueAlliance;

        onBlueAlliance = allianceSelector.getSelected() == Alliance.Blue;

        int selectedNodeIndex = nodeButtons.indexOf(storedSelectedButton);

        //Pose2d initialPosition = m_drivetrain.getPose();
        //ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        //Pose2d finalPosition;

        PathPlannerTrajectory trajectory;

        Rotation2d trajectoryHeading = Rotation2d.fromDegrees(onBlueAlliance? 180 : 0);

        Pose2d initialPosition = m_drivetrain.getPose();
        
        PathPoint initialPoint = new PathPoint(initialPosition.getTranslation(), Rotation2d.fromDegrees(90), initialPosition.getRotation());
        PathPoint secondPoint;
        ArrayList<PathPoint> remainingPoints = new ArrayList<PathPoint>();
        PathPoint[] remainingPointsFinal = new PathPoint[0];

        Pose2d finalPosition = nodePosition;

        //Command positionSequence;

        /*if(selectedNodeIndex < 9) {
            positionSequence = new ScoreHighSequence(m_elevator, m_arm);
        } else if(selectedNodeIndex < 18) {
            positionSequence = new GoToState(m_elevator, m_arm, Constants.kRobotStates.midScore);
        } else {
            positionSequence = new GoToState(m_elevator, m_arm, Constants.kRobotStates.lowScore);
        }*/

        if(((selectedNodeIndex == 0 || selectedNodeIndex == 1 ||
            selectedNodeIndex == 9 || selectedNodeIndex == 10 ||
            selectedNodeIndex == 18 || selectedNodeIndex == 19) &&
            (onBlueAlliance? initialPosition.getY() < 1.05 : initialPosition.getY() > 4.44 || 
            (initialPosition.getX() < 2.75 || initialPosition.getX() > 13.81))) ||
            ((selectedNodeIndex == 7 || selectedNodeIndex == 8 ||
            selectedNodeIndex == 16 || selectedNodeIndex == 17 ||
            selectedNodeIndex == 25 || selectedNodeIndex == 26) && 
            (onBlueAlliance? initialPosition.getY() > 4.44 : initialPosition.getY() < 1.05 ||
            (initialPosition.getX() < 2.75 || initialPosition.getX() > 13.81))))
            {
            //interiorWaypoints.add(new Translation2d((initialPosition.getX() + finalPosition.getX()) / 2, (initialPosition.getY() + finalPosition.getY()) / 2));
            secondPoint = new PathPoint(finalPosition.getTranslation(), trajectoryHeading, finalPosition.getRotation()).withPrevControlLength(0.75);
        } else {

            initialPoint = initialPoint.withNextControlLength(0.5);

            if(
                selectedNodeIndex == 0 || selectedNodeIndex == 1 ||
                selectedNodeIndex == 9 || selectedNodeIndex == 10 ||
                selectedNodeIndex == 18 || selectedNodeIndex == 19
                ) {
                //interiorWaypoints.add(new Translation2d(initialPosition.getX(), 0.8));
                //interiorWaypoints.add(new Translation2d(onBlueAlliance? 2.65 : 13.91, 0.8));
                secondPoint = new PathPoint(new Translation2d(initialPosition.getX(), onBlueAlliance? 0.8 : 4.62), trajectoryHeading, finalPosition.getRotation())
                .withControlLengths(0.5, 0.5);
            } else if(
                selectedNodeIndex == 2 || selectedNodeIndex == 3 || selectedNodeIndex == 4 ||
                selectedNodeIndex == 11 || selectedNodeIndex == 12 || selectedNodeIndex == 13 ||
                selectedNodeIndex == 20 || selectedNodeIndex == 21 || selectedNodeIndex == 22
                ) {
                secondPoint = new PathPoint(new Translation2d(initialPosition.getX(), onBlueAlliance? 0.8 : 4.62), trajectoryHeading, finalPosition.getRotation())
                .withControlLengths(0.25, 0.25);
                remainingPoints.add(new PathPoint(new Translation2d(onBlueAlliance? 2.65 : 13.91, onBlueAlliance? 0.8 : 4.62), Rotation2d.fromDegrees(-90), finalPosition.getRotation())
                .withControlLengths(0.5, 0.5));
            } else if(
                selectedNodeIndex == 8 || selectedNodeIndex == 7 ||
                selectedNodeIndex == 17 || selectedNodeIndex == 18 ||
                selectedNodeIndex == 26 || selectedNodeIndex == 27
                ) {
                secondPoint = new PathPoint(new Translation2d(initialPosition.getX(), onBlueAlliance? 4.62 : 0.8), trajectoryHeading, finalPosition.getRotation())
                .withControlLengths(0.5, 0.5);
            } else {
                //interiorWaypoints.add(new Translation2d(initialPosition.getX(), 4.62));
                //interiorWaypoints.add(new Translation2d(onBlueAlliance? 2.65 : 13.91, 4.62));
                secondPoint = new PathPoint(new Translation2d(onBlueAlliance? initialPosition.getX() - 0.6 : initialPosition.getX() + 0.6, onBlueAlliance? 4.62 : 0.8), trajectoryHeading, finalPosition.getRotation());
                remainingPoints.add(new PathPoint(new Translation2d(onBlueAlliance? 2.65 : 13.91, onBlueAlliance? 4.62 : 0.8), trajectoryHeading, finalPosition.getRotation()));
            }

            remainingPoints.add(new PathPoint(finalPosition.getTranslation(), trajectoryHeading, finalPosition.getRotation()));

            remainingPointsFinal = remainingPoints.toArray(remainingPointsFinal);

        }

        trajectory = PathPlanner.generatePath(
            new PathConstraints(Constants.kAutonomous.kMaxVelocityMetersPerSecond, Constants.kAutonomous.kMaxAccelerationMetersPerSecondSquared),
            initialPoint,
            secondPoint,
            remainingPointsFinal);

            return new SequentialCommandGroup(
                        new Field2dTrajectoryFollowerSequence(
                            m_drivetrain, 
                            trajectory
                            /*TrajectoryGenerator.generateTrajectory(
                                initialPosition, 
                                interiorWaypoints, 
                                finalPosition, 
                                new TrajectoryConfig(
                                    Constants.kAutonomous.kMaxVelocityMetersPerSecond, 
                                    Constants.kAutonomous.kMaxAccelerationMetersPerSecondSquared).
                                    setKinematics(Constants.kAutonomous.kSwerveKinematics))*/)/*,
                        positionSequence*/);


    }

    public static Pose2d getNodePosition() {

        return nodePosition;

    }

}