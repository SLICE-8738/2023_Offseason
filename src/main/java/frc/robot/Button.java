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
    public static Trigger setDrivePercentOutput = new JoystickButton(operatorJoystick, 3); //Left Top 3
    public static Trigger resetFieldOrientedHeading = new JoystickButton(driverController, 4); //Driver Y Button

    //Outtake Command Buttons
    public static Trigger outPutsConeOrCube = new JoystickButton(operatorJoystick, 1); //Left Top 1

    //Unassigned Left Joystick Buttons
    public static Trigger oepratorJoystickButton2 = new JoystickButton(operatorJoystick, 2); //Left Top 2
    public static Trigger oepratorJoystickButton4 = new JoystickButton(operatorJoystick, 4); //Left Top 4
    public static Trigger oepratorJoystickButton5 = new JoystickButton(operatorJoystick, 5); //Left Top 5
    public static Trigger oepratorJoystickButton6 = new JoystickButton(operatorJoystick, 6); //Left Top 6
    public static Trigger oepratorJoystickButton7 = new JoystickButton(operatorJoystick, 7); //Left Bottom 7
    public static Trigger oepratorJoystickButton8 = new JoystickButton(operatorJoystick, 8); //Left Bottom 8
    public static Trigger oepratorJoystickButton9 = new JoystickButton(operatorJoystick, 9); //Left Bottom 9
    public static Trigger oepratorJoystickButton10 = new JoystickButton(operatorJoystick, 10); //Left Bottom 10
    public static Trigger oepratorJoystickButton11 = new JoystickButton(operatorJoystick, 11); //Left Bottom 11
    public static Trigger oepratorJoystickButton12 = new JoystickButton(operatorJoystick, 12); //Left Bottom 12
    
    //Unassigned Driver Controller Buttons
    public static Trigger driverButton1 = new JoystickButton(driverController, 1); //Driver X Button
    public static Trigger driverButton2 = new JoystickButton(driverController, 2); //Driver A Button
    public static Trigger driverButton3 = new JoystickButton(driverController, 3); //Driver B Button
    public static Trigger driverButton5 = new JoystickButton(driverController, 5); //Driver Left Bumper
    public static Trigger driverButton6 = new JoystickButton(driverController, 6); //Driver Right Bumper
    public static Trigger driverButton7 = new JoystickButton(driverController, 7); //Driver Left Trigger
    public static Trigger driverButton8 = new JoystickButton(driverController, 8); //Driver Right Trigger
    public static Trigger driverButton9 = new JoystickButton(driverController, 9); //Driver Back Button
    public static Trigger driverButton10 = new JoystickButton(driverController, 10); //Driver Start Button
    public static Trigger driverButton11 = new JoystickButton(driverController, 11); //Driver Left Stick Push
    public static Trigger driverButton12 = new JoystickButton(driverController, 12); //Driver Right Stick Push

        //Unassigned Driver Controller Buttons
        public static Trigger operatorControllerButton1 = new JoystickButton(operatorController, 1); //Driver X Button
        public static Trigger operatorControllerButton2 = new JoystickButton(operatorController, 2); //Driver A Button
        public static Trigger operatorControllerButton3 = new JoystickButton(operatorController, 3); //Driver B Button
        public static Trigger operatorControllerButton4 = new JoystickButton(operatorController, 4); //Driver Y Button
        public static Trigger operatorControllerButton5 = new JoystickButton(operatorController, 5); //Driver Left Bumper
        public static Trigger operatorControllerButton6 = new JoystickButton(operatorController, 6); //Driver Right Bumper
        public static Trigger operatorControllerButton7 = new JoystickButton(operatorController, 7); //Driver Left Trigger
        public static Trigger operatorControllerButton8 = new JoystickButton(operatorController, 8); //Driver Right Trigger
        public static Trigger operatorControllerButton9 = new JoystickButton(operatorController, 9); //Driver Back Button
        public static Trigger operatorControllerButton10 = new JoystickButton(operatorController, 10); //Driver Start Button
        public static Trigger operatorControllerButton11 = new JoystickButton(operatorController, 11); //Driver Left Stick Push
        public static Trigger operatorControllerButton12 = new JoystickButton(operatorController, 12); //Driver Right Stick Push

}