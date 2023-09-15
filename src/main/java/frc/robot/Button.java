// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Button {

    //Joysticks
    public static Joystick operatorJoystick = new Joystick(Constants.kJoysticks.JOYSTICK_PORT);

    //Controllers
    public static GenericHID driverController = new GenericHID(Constants.kController.DRIVER_CONTROLLER_PORT);
    public static GenericHID operatorController = new GenericHID(Constants.kController.OPERATOR_CONTROLLER_PORT);

    //Drivetrain Command Buttons
    public static Trigger resetFieldOrientedHeading = new JoystickButton(driverController, 4); //Driver Triangle Button
    public static Trigger reverseFieldOrientedHeading = new JoystickButton(driverController, 1); //Driver Square Button
    public static Trigger setDrivePercentOutput = new JoystickButton(driverController, 6); //Driver Right Bumper
    public static Trigger slowModeLow = new JoystickButton(driverController, 7); //Driver Left Trigger
    public static Trigger slowModeHigh = new JoystickButton(driverController, 8); //Driver Right Trigger
    public static Trigger resetModuleAngles = new JoystickButton(driverController, 2); //Driver X Button
    
    //Stow
    public static Trigger stow = new JoystickButton(operatorJoystick, 1); //Operator Top 1

    //Intake State Buttons
    public static Trigger uprightConeGroundIntake = new JoystickButton(operatorJoystick, 3); //Operator Top 3
    public static Trigger cubeGroundIntake = new JoystickButton(operatorJoystick, 4); //Operator Top 5
    public static Trigger coneDoubleSubstationIntake = new JoystickButton(operatorJoystick, 5); //Operator Top 6
    public static Trigger cubeDoubleSubstationIntake = new JoystickButton(operatorJoystick, 6); //Operator Top 4
    public static Trigger coneSingleSubstationIntake = new JoystickButton(operatorJoystick, 2); //Operator Top 2

    //Score State Buttons
    public static Trigger scoreHigh = new JoystickButton(operatorJoystick, 7); //Operator Bottom 7
    public static Trigger scoreMid = new JoystickButton(operatorJoystick, 9); //Operator Bottom 9
    public static Trigger scoreLow = new JoystickButton(operatorJoystick, 11); //Operator Bottom 11

    //Manual Control
    public static Trigger wristUp = new JoystickButton(operatorController, 7); //Driver Left Trigger
    public static Trigger wristDown = new JoystickButton(operatorController, 8); //Driver Right Trigger

    //Outtake
    public static Trigger outtake2 = new JoystickButton(operatorJoystick, 12); //Operator Bottom 12
    public static Trigger controllerOuttake = new JoystickButton(operatorController, 2); //Operator A Button
    public static Trigger outtake = outtake2.or(controllerOuttake);
    public static Trigger intake = new JoystickButton(operatorController, 4); //Operator Y Button

    public static Trigger setToStart = new JoystickButton(operatorController, 3); //Operator B Button

    //Unassigned Driver Controller Buttons
    public static Trigger driverButton3 = new JoystickButton(driverController, 3); //Driver Circle Button
    public static Trigger driverButton5 = new JoystickButton(driverController, 5); //Driver Left Bumper
    public static Trigger driverButton9 = new JoystickButton(driverController, 9); //Driver Share Button
    public static Trigger driverButton10 = new JoystickButton(driverController, 10); //Driver Options Button
    public static Trigger driverButton11 = new JoystickButton(driverController, 11); //Driver Left Stick Push
    public static Trigger driverButton12 = new JoystickButton(driverController, 12); //Driver Right Stick Push
    public static Trigger driverButton13 = new JoystickButton(driverController, 13); //Driver Playstation Button
    public static Trigger driverButton14 = new JoystickButton(driverController, 14); //Driver Touchpad Push

    //Unassigned Operator Controller Buttons
    public static Trigger operatorControllerButton2 = new JoystickButton(operatorController, 1); //Operator X Button
    public static Trigger operatorControllerButton5 = new JoystickButton(operatorController, 5); //Operator Left Bumper
    public static Trigger operatorControllerButton6 = new JoystickButton(operatorController, 6); //Operator Right Bumper
    public static Trigger operatorControllerButton9 = new JoystickButton(operatorController, 9); //Operator Back Button
    public static Trigger operatorControllerButton10 = new JoystickButton(operatorController, 10); //Operator Start Button
    public static Trigger operatorControllerButton11 = new JoystickButton(operatorController, 11); //Operator Left Stick Push
    public static Trigger operatorControllerButton12 = new JoystickButton(operatorController, 12); //Operator Right Stick Push

    //Unassigned Operator Joystick Buttons
    public static Trigger operatorJoystickButton8 = new JoystickButton(operatorJoystick, 8); //Operator Bottom 8
    public static Trigger operatorJoystickButton10 = new JoystickButton(operatorJoystick, 10); //Operator Bottom 10

}