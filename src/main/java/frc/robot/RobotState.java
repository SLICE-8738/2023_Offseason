// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * A simple data class representing the state of the robot.
 */
public class RobotState {
    private final double elevatorHeight, elbowAngle, wristAngle;

    public RobotState(double elevatorHeight, double elbowAngle, double wristAngle) {
        this.elevatorHeight = elevatorHeight;
        this.elbowAngle = elbowAngle;
        this.wristAngle = wristAngle;
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public double getElbowAngle() {
        return elbowAngle;
    }

    public double getWristAngle() {
        return wristAngle;
    }
}
