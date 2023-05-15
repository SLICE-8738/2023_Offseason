// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

  private final NetworkTable table;
  
  private double targetDetected;

  private double targetXOffset;
  private double targetYOffset;

  private static double[] currentBotPoseBlue;
  private static double[] lastNonNullBotPoseBlue = new double[0];
  private static double[] lastNonEmptyBotPoseBlue = new double[0];
  private static double[] lastBotPoseTargetSpace = new double[0];
  private static double[] currentBotPoseTargetSpace;

  private static double currentAprilTagID = 0;
  private static double lastDoubleSubAprilTagID = 0;
  private static double lastNodeAprilTagID = 0;

  private final NetworkTableEntry ledMode;
  private final NetworkTableEntry cameraMode;
  private final NetworkTableEntry pipeline;

  private final ShuffleboardTab driverTab;

  /** Creates a new Limelight. */
  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight-slice");

    ledMode = table.getEntry("ledMode");
    cameraMode = table.getEntry("cameraMode");
    pipeline = table.getEntry("pipeline");

    driverTab = Shuffleboard.getTab("Driver Tab");

    driverTab.addCamera("Limelight", "limelight-slice-1", "http://10.87.38.73:5800").
    withPosition(6, 0).
    withSize(3, 2);

    pipeline.setNumber(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    targetDetected = table.getEntry("tv").getDouble(0);

    targetXOffset = table.getEntry("tx").getDouble(0);
    targetYOffset = table.getEntry("ty").getDouble(0);

    currentBotPoseBlue = table.getEntry("botpose").getDoubleArray(new double[6]);

    if(currentBotPoseBlue != null) {

      lastNonNullBotPoseBlue = currentBotPoseBlue;

      if(lastNonNullBotPoseBlue.length != 0) {

        lastNonEmptyBotPoseBlue = lastNonNullBotPoseBlue;
    
      }

    }

    
    currentBotPoseTargetSpace = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    //currentBotPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight-slice");

    if(currentBotPoseTargetSpace != null) {
    
      if(currentBotPoseTargetSpace.length != 0) {

        SmartDashboard.putNumber("Bot Pose Target Space X", currentBotPoseTargetSpace[5]);

      }

    }

    if(currentBotPoseTargetSpace != null) {

      lastBotPoseTargetSpace = currentBotPoseTargetSpace;

    }

    currentAprilTagID = table.getEntry("tid").getDouble(0);

    if(currentAprilTagID == 4 || currentAprilTagID == 5) {

      lastDoubleSubAprilTagID = currentAprilTagID;

    }

    if(currentAprilTagID != 4 && currentAprilTagID != 5 && currentAprilTagID != 0) {

      lastNodeAprilTagID = currentAprilTagID;

    }

    SmartDashboard.putNumber("Last Apriltag ID", lastDoubleSubAprilTagID);

  }

  public double getTargetDetected() {

    return targetDetected;

  }

  public double getXOffset() {

    return targetXOffset;

  }

  public double getYOffset() {

    return targetYOffset;

  }

  public static Pose2d getLastBotPoseBlue() {

    double[] lastNonEmptyBotPoseBlue = Limelight.lastNonEmptyBotPoseBlue;

    if(lastNonEmptyBotPoseBlue.length != 0) {

      return new Pose2d(lastNonEmptyBotPoseBlue[0] + 8.28, lastNonEmptyBotPoseBlue[1] + 4, Rotation2d.fromDegrees(lastNonEmptyBotPoseBlue[5]));

    }
    else {

      return new Pose2d(8.28, 4, Rotation2d.fromDegrees(0));

    }

  }

  public static Pose2d getCurrentBotPoseBlue() {

    double[] lastNonNullBotPoseBlue = Limelight.lastNonNullBotPoseBlue;

    if(lastNonNullBotPoseBlue.length != 0) {
      
      return new Pose2d(lastNonNullBotPoseBlue[0] + 8.28, lastNonNullBotPoseBlue[1] + 4, Rotation2d.fromDegrees(lastNonNullBotPoseBlue[5]));
  
    }
    else {

      return null;

    }

  }

  public static Pose2d getBotPoseTargetSpace() {

    double[] lastBotPoseTargetSpace = Limelight.lastBotPoseTargetSpace;

    if(lastBotPoseTargetSpace.length != 0) {

      return new Pose2d(lastBotPoseTargetSpace[2], lastBotPoseTargetSpace[0], Rotation2d.fromDegrees(lastBotPoseTargetSpace[5]));

    }
    else {

      return null;

    }

  }

  public double getAprilTagID() {

    return currentAprilTagID;

  }

  public void setLedMode(Number mode) {

    ledMode.setNumber(mode);

  }
  
  public void setCameraMode(Number mode) {

    cameraMode.setNumber(mode);

  }

  public void setPipeline(Number pipelineNumber) {

    pipeline.setNumber(pipelineNumber);

  }

  public static Trajectory generateDoubleSubstationTrajectory(Pose2d initialPosition) {

    double aprilTagX;
    double aprilTagY;

    Pose2d finalPosition;

    double lastDoubleSubAprilTagID = Limelight.lastDoubleSubAprilTagID;

    if(lastDoubleSubAprilTagID != 0) {
 
      if(lastDoubleSubAprilTagID == 4) {

        aprilTagX = 16.19;
        aprilTagY = 6.74;
        finalPosition = new Pose2d(aprilTagX - 2, aprilTagY + 0.6, Rotation2d.fromDegrees(0));
  
      }
      else {
  
        aprilTagX = 0.37;
        aprilTagY = 6.74;
        finalPosition = new Pose2d(aprilTagX + 2, aprilTagY + 0.6, Rotation2d.fromDegrees(180));
  
      }

      return TrajectoryGenerator.generateTrajectory(
        initialPosition, 
        List.of(new Translation2d(
          (initialPosition.getX() + finalPosition.getX()) / 2, 
          (initialPosition.getY() + finalPosition.getY()) / 2)),
        finalPosition, 
        new TrajectoryConfig(1, 0.5).setKinematics(Constants.Drivetrain.kDriveKinematics));

    }
    else {

      return new Trajectory(List.of(new State(0, 0, 0, initialPosition, 0)));

    }

  }

  public static Trajectory generateNodeTrajectory(Pose2d initialPosition) {

    double aprilTagX;
    double aprilTagY;

    Translation2d interiorWaypoint = new Translation2d(0, 0);
    Pose2d finalPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    double lastNodeAprilTagID = Limelight.lastNodeAprilTagID;

    if(lastNodeAprilTagID != 0) {

      if(lastNodeAprilTagID == 1) {

        aprilTagX = 15.52;
        aprilTagY = 1.06;

        finalPosition = new Pose2d(aprilTagX - 0.8, aprilTagY, Rotation2d.fromDegrees(0));

        interiorWaypoint = new Translation2d(
          (initialPosition.getX() + finalPosition.getX()) / 2, 
          (initialPosition.getY() + finalPosition.getY()) / 2);

      }
      else if(lastNodeAprilTagID == 2) {

        aprilTagX = 15.52;
        aprilTagY = 2.74;

        finalPosition = new Pose2d(aprilTagX - 0.8, aprilTagY, Rotation2d.fromDegrees(0));

        interiorWaypoint = new Translation2d(13.91, initialPosition.getY());

      }
      else if (lastNodeAprilTagID == 3) {

        aprilTagX = 15.52;
        aprilTagY = 4.42;

        finalPosition = new Pose2d(aprilTagX - 0.8, aprilTagY, Rotation2d.fromDegrees(0));

        interiorWaypoint = new Translation2d(
          (initialPosition.getX() + finalPosition.getX()) / 2, 
          (initialPosition.getY() + finalPosition.getY()) / 2);

      }
      else if(lastNodeAprilTagID == 6) {

        aprilTagX = 1.04;
        aprilTagY = 4.42;

        finalPosition = new Pose2d(aprilTagX + 0.8, aprilTagY, Rotation2d.fromDegrees(180));

        interiorWaypoint = new Translation2d(
          (initialPosition.getX() + finalPosition.getX()) / 2, 
          (initialPosition.getY() + finalPosition.getY()) / 2);

      }
      else if(lastNodeAprilTagID == 7) {

        aprilTagX = 1.04;
        aprilTagY = 2.74;

        finalPosition = new Pose2d(aprilTagX + 0.8, aprilTagY, Rotation2d.fromDegrees(180));

        interiorWaypoint = new Translation2d(2.65, initialPosition.getY());

      }
      else {

        aprilTagX = 1.04;
        aprilTagY = 1.06;

        finalPosition = new Pose2d(aprilTagX + 0.8, aprilTagY, Rotation2d.fromDegrees(180));

        interiorWaypoint = new Translation2d(
          (initialPosition.getX() + finalPosition.getX()) / 2, 
          (initialPosition.getY() + finalPosition.getY()) / 2);

      }

      return TrajectoryGenerator.generateTrajectory(
        initialPosition, 
        List.of(interiorWaypoint),
        finalPosition, 
        new TrajectoryConfig(1, 0.5).setKinematics(Constants.Drivetrain.kDriveKinematics));

    }
    else {

      return new Trajectory(List.of(new State(0, 0, 0, initialPosition, 0)));

    }

  }

}