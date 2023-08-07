// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.Drivetrain;

/**
 * This class should be used to statically call its methods in order to create
 * {@link SwerveControllerCommand} or {@link PPSwerveControllerCommand} objects, which are used to allow the robot to 
 * drive autonomously.
 * 
 */
@SuppressWarnings("resource")
public class TrajectoryCommands {

    public static SwerveControllerCommand generateSwerveControllerCommand(Drivetrain drive, Trajectory trajectory, boolean useTrajectoryRotations) {

        if(useTrajectoryRotations) {

            return new SwerveControllerCommand(
                trajectory,
                drive::getPose,
                Constants.kDrivetrain.kSwerveKinematics,
                    new PIDController(Constants.kAutonomous.kPXController, 0, 0),
                    new PIDController(Constants.kAutonomous.kPYController, 0, 0),
                    new ProfiledPIDController(
                        Constants.kAutonomous.kPThetaController,
                        0,
                        0,
                        Constants.kAutonomous.kThetaControllerConstraints),
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
                    new PIDController(Constants.kAutonomous.kPXController, 0, 0),
                    new PIDController(Constants.kAutonomous.kPYController, 0, 0),
                    new ProfiledPIDController(
                        Constants.kAutonomous.kPThetaController,
                        0,
                        0,
                        Constants.kAutonomous.kThetaControllerConstraints),
                //SwerveControllerCommand passes output module states to the callback
                drive::setModuleStates,
                drive);

        }

    }

    public static PPSwerveControllerCommand generatePPSwerveControllerCommand(Drivetrain drive, PathPlannerTrajectory trajectory) {

        return new PPSwerveControllerCommand(
            trajectory,
            drive::getPose,
            Constants.kDrivetrain.kSwerveKinematics,
                new PIDController(Constants.kAutonomous.kPXController, 0, 0),
                new PIDController(Constants.kAutonomous.kPYController, 0, 0),
                new PIDController(Constants.kAutonomous.kPThetaController, 0, 0),
            //SwerveControllerCommand passes output module states to the callback
            drive::setModuleStates,
            drive);

    }

    public static Trajectory generateTestTrajectory(Drivetrain drive, Transform2d trajectoryTransform) {

        Pose2d initialPosition = drive.getPose();
        Pose2d finalPosition = initialPosition.plus(trajectoryTransform);

        Translation2d interiorWaypoint = initialPosition.getTranslation().plus(trajectoryTransform.getTranslation().div(2));

        return TrajectoryGenerator.generateTrajectory(
                    drive.getPose(), 
                    List.of(interiorWaypoint), 
                    finalPosition, 
                    new TrajectoryConfig(
                        4, 
                        2).
                        setKinematics(Constants.kDrivetrain.kSwerveKinematics));

    }

}
