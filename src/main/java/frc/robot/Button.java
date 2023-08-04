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
    public static Joystick leftJoystick = new Joystick(Constants.kJoysticks.LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(Constants.kJoysticks.RIGHT_JOYSTICK_PORT);

    //Controllers
    public static GenericHID driverController = new GenericHID(Constants.kController.DRIVER_CONTROLLER_PORT);

    //Drivetrain Command Buttons
    public static Trigger setDrivePercentOutput = new JoystickButton(leftJoystick, 3); //Left Top 3
    public static Trigger resetFieldOrientedHeading = new JoystickButton(driverController, 4); //Driver Y Button

    //Outtake Command Buttons
    public static Trigger outPutsConeOrCube = new JoystickButton(leftJoystick, 1); //Left Top 1

    //Unassigned Left Joystick Buttons
    public static Trigger leftButton2 = new JoystickButton(leftJoystick, 2); //Left Top 2
    public static Trigger leftButton4 = new JoystickButton(leftJoystick, 4); //Left Top 4
    public static Trigger leftButton5 = new JoystickButton(leftJoystick, 5); //Left Top 5
    public static Trigger leftButton6 = new JoystickButton(leftJoystick, 6); //Left Top 6
    public static Trigger leftButton7 = new JoystickButton(leftJoystick, 7); //Left Bottom 7
    public static Trigger leftButton8 = new JoystickButton(leftJoystick, 8); //Left Bottom 8
    public static Trigger leftButton9 = new JoystickButton(leftJoystick, 9); //Left Bottom 9
    public static Trigger leftButton10 = new JoystickButton(leftJoystick, 10); //Left Bottom 10
    public static Trigger leftButton11 = new JoystickButton(leftJoystick, 11); //Left Bottom 11
    public static Trigger leftButton12 = new JoystickButton(leftJoystick, 12); //Left Bottom 12

    //Unassigned Right Joystick Buttons
    public static Trigger rightButton1 = new JoystickButton(rightJoystick, 1); //Right Top 1
    public static Trigger rightButton2 = new JoystickButton(rightJoystick, 2); //Right Top 2
    public static Trigger rightButton3 = new JoystickButton(rightJoystick, 3); //Right Top 3
    public static Trigger rightButton4 = new JoystickButton(rightJoystick, 4); //Right Top 4
    public static Trigger rightButton5 = new JoystickButton(rightJoystick, 5); //Right Top 5
    public static Trigger rightButton6 = new JoystickButton(rightJoystick, 6); //Right Top 6
    public static Trigger rightButton7 = new JoystickButton(rightJoystick, 7); //Right Bottom 7
    public static Trigger rightButton8 = new JoystickButton(rightJoystick, 8); //Right Bottom 8
    public static Trigger rightButton9 = new JoystickButton(rightJoystick, 9); //Right Bottom 9
    public static Trigger rightButton10 = new JoystickButton(rightJoystick, 10); //Right Bottom 10
    public static Trigger rightButton11 = new JoystickButton(rightJoystick, 11); //Right Bottom 11
    public static Trigger rightButton12 = new JoystickButton(rightJoystick, 12); //Right Bottom 12   
    
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

}