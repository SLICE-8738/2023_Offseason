// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.Drivetrain;

/**
 * This class should be used to statically call its methods in order to create
 * {@link SwerveControllerCommand} or {@link PPSwerveControllerCommand} objects, which are used to allow the robot to 
 * drive autonomously.
 * 
 */
public class TrajectoryCommands {

    private static final PIDController xController = new PIDController(Constants.kAutonomous.kPXController, 0, 0);
    private static final PIDController yController = new PIDController(Constants.kAutonomous.kPYController, 0, 0);

    private static final ProfiledPIDController thetaProfiledController = new ProfiledPIDController(
        Constants.kAutonomous.kPThetaController,
        0,
        0,
        Constants.kAutonomous.kThetaControllerConstraints);
    private static final PIDController thetaController = new PIDController(Constants.kAutonomous.kPThetaController, 0, 0);

    public static SwerveControllerCommand generateSwerveControllerCommand(Drivetrain drive, Trajectory trajectory, boolean useTrajectoryRotations) {

        if(useTrajectoryRotations) {

            return new SwerveControllerCommand(
                trajectory,
                drive::getPose,
                Constants.kDrivetrain.kSwerveKinematics,
                xController,
                yController,
                thetaProfiledController,
                drive::getAutoTrajectoryRotation,
                //SwerveControllerCommand passes output module states to the callback
                drive::setModuleStates,
                drive);
        }
        else {

            return new SwerveControllerCommand(
                trajectory,
                drive::getPose,
                Constants.kDrivetrain.kSwerveKinematics,
                xController,
                yController,
                thetaProfiledController,
                //SwerveControllerCommand passes output module states to the callback
                drive::setModuleStates,
                drive);

        }

    }

    public static PPSwerveControllerCommand generatePPSwerveControllerCommand(Drivetrain drive, PathPlannerTrajectory trajectory, boolean useAllianceColor) {

        return new PPSwerveControllerCommand(
            trajectory,
            drive::getPose,
            Constants.kDrivetrain.kSwerveKinematics,
                xController,
                yController,
                thetaController,
            //SwerveControllerCommand passes output module states to the callback
            drive::setModuleStates,
            useAllianceColor, 
            drive);

    }

    public static PathPlannerTrajectory generateTestTrajectory(Drivetrain drive, Transform2d trajectoryTransform) {

        /*Pose2d initialPosition = drive.getPose();
        Pose2d finalPosition = initialPosition.plus(trajectoryTransform);

        Translation2d interiorWaypoint = initialPosition.getTranslation().plus(trajectoryTransform.getTranslation().div(2));

        return TrajectoryGenerator.generateTrajectory(
                    initialPosition, 
                    List.of(interiorWaypoint), 
                    finalPosition, 
                    new TrajectoryConfig(
                        Constants.kAutonomous.kMaxVelocityMetersPerSecond, 
                        Constants.kAutonomous.kMaxAccelerationMetersPerSecondSquared).
                        setKinematics(Constants.kAutonomous.kSwerveKinematics));*/

        Rotation2d trajectoryHeading = new Rotation2d(trajectoryTransform.getX(), trajectoryTransform.getY());

        Pose2d initialPosition = drive.getPose();
        Pose2d finalPosition = initialPosition.plus(trajectoryTransform);

        PathPoint initialPoint = new PathPoint(initialPosition.getTranslation(), trajectoryHeading, initialPosition.getRotation());
        PathPoint finalPoint = new PathPoint(finalPosition.getTranslation(), trajectoryHeading, finalPosition.getRotation());

        return PathPlanner.generatePath(
            new PathConstraints(Constants.kAutonomous.kMaxVelocityMetersPerSecond, Constants.kAutonomous.kMaxAccelerationMetersPerSecondSquared),
            initialPoint,
            finalPoint);

    }

}