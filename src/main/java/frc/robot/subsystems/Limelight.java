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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

//*Creates Limelight subsystem */
public class Limelight extends SubsystemBase {

  //Creates Network table table
  private final NetworkTable table;

  private double targetDetected;

  private double targetXOffset;
  private double targetYOffset;

  private static double[] currentBotPose;
  private static double[] currentBotPoseBlue;
  private static double[] lastNonNullBotPoseBlue = new double[0];
  private static double[] lastNonEmptyBotPoseBlue = {8.28, 4, 0, 0, 0, 0};
  private static double[] lastBotPoseTargetSpace = new double[0];
  private static double[] currentBotPoseTargetSpace;

  //Stores april tags 
  private static double currentAprilTagID;
  private static double lastDoubleSubAprilTagID;

  private final NetworkTableEntry ledMode;
  //Creates mode where you can use limelight as a camera 
  private final NetworkTableEntry cameraMode;
  private final NetworkTableEntry pipeline;

  /** Creates a new Limelight. */
  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight-slice");

    //adds diffrent modes for the robot to be in
    ledMode = table.getEntry("ledMode");
    //camera mode lets limelight function as a cammera instead of seach8ing for april tags 
    cameraMode = table.getEntry("cameraMode");

    pipeline = table.getEntry("pipeline");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    targetDetected = table.getEntry("tv").getDouble(0);

    targetXOffset = table.getEntry("tx").getDouble(0);
    targetYOffset = table.getEntry("ty").getDouble(0);

    currentBotPose = table.getEntry("botpose").getDoubleArray(new double[6]);

    if(currentBotPose != null) {

      if(currentBotPose.length != 0) {

        currentBotPose[0] += 8.28;
        currentBotPose[1] += 4;
        
        currentBotPoseBlue = currentBotPose;

      }

    }

    if(currentBotPoseBlue != null) {

      lastNonNullBotPoseBlue = currentBotPoseBlue;

      if(lastNonNullBotPoseBlue.length != 0) {

        lastNonEmptyBotPoseBlue = lastNonNullBotPoseBlue;

      }

    }

    currentBotPoseTargetSpace = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    if(currentBotPoseTargetSpace != null) {

      lastBotPoseTargetSpace = currentBotPoseTargetSpace;

    }

    currentAprilTagID = table.getEntry("tid").getDouble(0);

    if(currentAprilTagID == 4 || currentAprilTagID == 5) {

       lastDoubleSubAprilTagID = currentAprilTagID;

    }

    SmartDashboard.putNumber("Last Double Sub Apriltag ID", lastDoubleSubAprilTagID);

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

    return new Pose2d(lastNonEmptyBotPoseBlue[0], lastNonEmptyBotPoseBlue[1], Rotation2d.fromDegrees(lastNonEmptyBotPoseBlue[5]));

  }

  public static Pose2d getCurrentBotPoseBlue() {

    double[] lastNonNullBotPoseBlue = Limelight.lastNonNullBotPoseBlue;

    if(lastNonNullBotPoseBlue.length != 0) {

      return new Pose2d(lastNonNullBotPoseBlue[0], lastNonNullBotPoseBlue[1], Rotation2d.fromDegrees(lastNonNullBotPoseBlue[5]));

    }
    else {

      return null;

    }

  }

  public static Pose2d getBotPoseTargetSpace() {

    double[] lastBotPoseTargetSpace = Limelight.lastBotPoseTargetSpace;

    if(lastBotPoseTargetSpace.length != 0) {

      return new Pose2d(lastBotPoseTargetSpace[0], lastBotPoseTargetSpace[1], Rotation2d.fromDegrees(lastBotPoseTargetSpace[5]));

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

    //double lastDoubleSubAprilTagID = Limelight.lastDoubleSubAprilTagID;

    if(lastDoubleSubAprilTagID == 4 || lastDoubleSubAprilTagID == 5) {

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
        new TrajectoryConfig(4, 2).setKinematics(Constants.kDrivetrain.kSwerveKinematics));

    }
    else {

      return new Trajectory(
        List.of(new State(
          0, 
          0, 
          0, 
          initialPosition, 
          0)));

    }

  }

}