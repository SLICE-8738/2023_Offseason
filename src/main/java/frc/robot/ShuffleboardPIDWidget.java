// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.Map;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** 
 * This represents a custom widget on shuffleboard that accepts a P, I, and D gain
 * and updates the provided PID controler to those gains when a button in the widget is pressed.
 */
public class ShuffleboardPIDWidget {
    private ShuffleboardTab tab;
    //private ShuffleboardLayout list;

    private SimpleWidget pGainWidget, iGainWidget, dGainWidget;

    private PIDController wpiLibPIDController;
    private ProfiledPIDController wpiLibProfiledPIDController;
    private SparkMaxPIDController sparkMaxPIDController;

    /**
     * Creates a new PID tuning widget for a {@link SparkMaxPIDController}.
     * 
     * @param sparkMaxPIDController The PID controller to be controlled by the widget.
     */
    public ShuffleboardPIDWidget(/*String name,*/ SparkMaxPIDController sparkMaxPIDController) {
        this.sparkMaxPIDController = sparkMaxPIDController;

        tab = Shuffleboard.getTab("PID Tuning");
        // A grid layout is used here rather than a list in order to control the order of the items
        /*list = tab.getLayout(name, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 4));*/
    
        pGainWidget = tab.add("P", 0.0)
            .withPosition(3, 1);

        iGainWidget = tab.add("I", 0.0)
            .withPosition(4, 1);

        dGainWidget = tab.add("D", 0.0)
            .withPosition(5, 1);

        tab.add("Update", new InstantCommand(this::updateSparkMaxGains))
            .withPosition(4, 2)
            .withWidget(BuiltInWidgets.kCommand);
    }

    /**
     * Creates a new PID tuning widget for a WPILib {@link PIDController}.
     * 
     * @param wpiLibPIDController The PID controller to be controlled by the widget.
     */
    public ShuffleboardPIDWidget(/*String name,*/ PIDController wpiLibPIDController) {
        this.wpiLibPIDController = wpiLibPIDController;

        tab = Shuffleboard.getTab("PID Tuning");
        // A grid layout is used here rather than a list in order to control the order of the items
        /*list = tab.getLayout(name, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 4));*/
    
        pGainWidget = tab.add("P", 0.0)
            .withPosition(3, 1);

        iGainWidget = tab.add("I", 0.0)
            .withPosition(4, 1);

        dGainWidget = tab.add("D", 0.0)
            .withPosition(5, 1);

        tab.add("Update", new InstantCommand(this::updateWPILibGains))
            .withPosition(4, 2)
            .withWidget(BuiltInWidgets.kCommand);
    }

    /**
     * Creates a new PID tuning widget for a WPILib {@link ProfiledPIDController}.
     * 
     * @param wpiLibPIDController The PID controller to be controlled by the widget.
     */
    public ShuffleboardPIDWidget(/*String name,*/ ProfiledPIDController wpiLibProfiledPIDController) {
        this.wpiLibProfiledPIDController = wpiLibProfiledPIDController;

        tab = Shuffleboard.getTab("PID Tuning");
        // A grid layout is used here rather than a list in order to control the order of the items
        /*list = tab.getLayout(name, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 4));*/
    
        pGainWidget = tab.add("P", 0.0)
            .withPosition(3, 1);

        iGainWidget = tab.add("I", 0.0)
            .withPosition(4, 1);

        dGainWidget = tab.add("D", 0.0)
            .withPosition(5, 1);

        tab.add("Update", new InstantCommand(this::updateWPILibProfiledGains))
            .withPosition(4, 2)
            .withWidget(BuiltInWidgets.kCommand);
    }

    public void updateWPILibGains() {
        wpiLibPIDController.setP(pGainWidget.getEntry().getDouble(0.0));
        wpiLibPIDController.setI(iGainWidget.getEntry().getDouble(0.0));
        wpiLibPIDController.setD(dGainWidget.getEntry().getDouble(0.0));
    }

    public void updateWPILibProfiledGains() {
        wpiLibProfiledPIDController.setP(pGainWidget.getEntry().getDouble(0.0));
        wpiLibProfiledPIDController.setI(iGainWidget.getEntry().getDouble(0.0));
        wpiLibProfiledPIDController.setD(dGainWidget.getEntry().getDouble(0.0));
    }

    public void updateSparkMaxGains() {
        sparkMaxPIDController.setP(pGainWidget.getEntry().getDouble(0.0));
        sparkMaxPIDController.setI(iGainWidget.getEntry().getDouble(0.0));
        sparkMaxPIDController.setD(dGainWidget.getEntry().getDouble(0.0));
    }
}