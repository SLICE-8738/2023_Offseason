// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** 
 * This represents a custom widget on shuffleboard that accepts a P, I, and D hain
 * and updates the provided PID controler to those gains when a button in the widget is pressed
 */
public class ShuffleboardPIDWidget {
    private ShuffleboardTab tab;
    private ShuffleboardLayout list;

    private SimpleWidget pGainWidget, iGainWidget, dGainWidget;

    private SparkMaxPIDController pidController;

    public ShuffleboardPIDWidget(String name, SparkMaxPIDController pidController) {
        this.pidController = pidController;

        tab = Shuffleboard.getTab("PID Tuning");
        // A grid layout is used here rather than a list in order to control the order of the items
        list = tab.getLayout(name, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 4))
            .withSize(2, 4);
    
        pGainWidget = tab.add("P", Constants.kDrivetrain.ANGLE_KP)
            .withPosition(0, 0);

        iGainWidget = tab.add("I", Constants.kDrivetrain.ANGLE_KI)
            .withPosition(0, 1);

        dGainWidget = tab.add("D", Constants.kDrivetrain.ANGLE_KD)
            .withPosition(0, 2);

        tab.add("Update", new InstantCommand(this::updateGains))
            .withPosition(0, 3)
            .withWidget(BuiltInWidgets.kCommand);
    }

    public void updateGains() {
        pidController.setP(pGainWidget.getEntry().getDouble(0.0));
        pidController.setI(iGainWidget.getEntry().getDouble(0.0));
        pidController.setP(dGainWidget.getEntry().getDouble(0.0));
    }
}
