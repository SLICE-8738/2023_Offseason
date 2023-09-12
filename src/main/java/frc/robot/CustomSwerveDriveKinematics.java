// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class CustomSwerveDriveKinematics extends SwerveDriveKinematics {

    public CustomSwerveDriveKinematics(Translation2d... wheelsMeters) {

        super(wheelsMeters);

    }

    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        return super.toSwerveModuleStates(newChassisSpeeds);
    }
}