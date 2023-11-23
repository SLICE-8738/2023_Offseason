// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.Drivetrain;

/**
 * This class should be used to statically call its methods in order to create
 * {@link SwerveControllerCommand} or {@link PPSwerveControllerCommand} objects, which are used to allow the robot to 
 * drive autonomously.
 * 
 */
public class TrajectoryCommands {

    private static final PIDController xController = new PIDController(Constants.kAutonomous.kPTranslation, 0, 0);
    private static final PIDController yController = new PIDController(Constants.kAutonomous.kPTranslation, 0, 0);
    private static final ProfiledPIDController thetaProfiledController = new ProfiledPIDController(
        Constants.kAutonomous.kPRotation,
        0,
        0,
        Constants.kAutonomous.kThetaControllerConstraints);

    private static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.kAutonomous.kPTranslation),
        new PIDConstants(Constants.kAutonomous.kPRotation),
        Constants.kAutonomous.kMaxVelocityMetersPerSecond,
        Constants.kDrivetrain.DRIVE_BASE_RADIUS,
        new ReplanningConfig(false, false));

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

    public static FollowPathHolonomic generateFollowPathHolonomicCommand(Drivetrain drive, PathPlannerPath path, boolean useAllianceColor) {

        return new FollowPathHolonomic(
            path,
            drive::getPose,
            drive::getChassisSpeeds,
            drive::setChassisSpeeds,
            pathFollowerConfig,
            drive);

    }

    public static PathfindHolonomic generatePathFindCommand(Drivetrain drive, Pose2d targetPose) {

        return new PathfindHolonomic(
            targetPose,
            Constants.kAutonomous.kPathConstraints,
            drive::getPose,
            drive::getChassisSpeeds,
            drive::setChassisSpeeds,
            pathFollowerConfig,
            drive);

    }

}