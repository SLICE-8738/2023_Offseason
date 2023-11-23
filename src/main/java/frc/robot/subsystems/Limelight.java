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

  private static double[] currentBotPoseBlue;
  private static double[] lastNonNullBotPoseBlue = new double[0];
  private static double[] lastNonEmptyBotPoseBlue;
  private double[] lastRobotTargetSpacePose;
  private double[] currentRobotTargetSpacePose;

  //Stores april tags 
  private static double currentAprilTagID;
  private static double lastAprilTagID;

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

    currentBotPoseBlue = table.getEntry("botpose").getDoubleArray(new double[6]);

    if(currentBotPoseBlue != null) {

      lastNonNullBotPoseBlue = currentBotPoseBlue;

      if(lastNonNullBotPoseBlue.length != 0) {

        lastNonEmptyBotPoseBlue = lastNonNullBotPoseBlue;
        
      }

    }

    currentRobotTargetSpacePose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    if(currentRobotTargetSpacePose != null) {

      lastRobotTargetSpacePose = currentRobotTargetSpacePose;

    }

    currentAprilTagID = table.getEntry("tid").getDouble(0);

    if(currentAprilTagID == 4 || currentAprilTagID == 5) {

       lastAprilTagID = currentAprilTagID;

    }

    SmartDashboard.putNumber("Last Apriltag ID", lastAprilTagID);

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

    if(lastNonEmptyBotPoseBlue != null) {

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

  public Pose2d getRobotTargetSpacePose() {

    return new Pose2d(lastRobotTargetSpacePose[0], lastRobotTargetSpacePose[1], Rotation2d.fromDegrees(lastRobotTargetSpacePose[5]));

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

    //double lastAprilTagID = Limelight.lastAprilTagID;

    if(lastAprilTagID == 4 || lastAprilTagID == 5) {

      if(lastAprilTagID == 4) {

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
        new TrajectoryConfig(0.5, 0.2).setKinematics(Constants.kDrivetrain.kSwerveKinematics));
  
    }
    else {
      
      return new Trajectory(List.of(new State(0, 0, 0, initialPosition, 0)));

    }

  }

}