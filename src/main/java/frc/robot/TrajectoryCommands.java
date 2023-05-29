// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.Drivetrain;

/**
 * This class should be used to statically call its methods in order to create
 * {@link SwerveControllerCommand} objects, which are used to allow the robot to 
 * drive during autonomous.
 * 
 */
public class TrajectoryCommands {

    public static SwerveControllerCommand generateSwerveControllerCommand(Drivetrain drive, Trajectory trajectory) {

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

}
