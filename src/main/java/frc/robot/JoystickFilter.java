// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ServiceConfigurationError;

import edu.wpi.first.math.MathUtil;

/**
 * A class to filter the input of a joystick axis.
 */
public class JoystickFilter {
    private double lastInput;
    private final double deadzone, smoothing;

    /**
     * @param deadzone input from the joystick that is less than the deadzone will be ignored. -1 to 1
     * @param smoothing a higher value will smooth the input more but also increase input delay. 0 to 1
     */
    public JoystickFilter(double deadzone, double smoothing) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        
        lastInput = 0;
    }

    public double withDead(double raw) {
        return MathUtil.applyDeadband(raw, deadzone);
    }

    public double withCurve(double raw) {
        double firstTerm = Constants.Joysticks.A_COEFFICIENT * Math.pow(raw, Constants.Joysticks.FIRST_POWER);
        double secondTerm = Constants.Joysticks.B_COEFFICIENT * Math.pow(raw, Constants.Joysticks.SECOND_POWER);
        return firstTerm + secondTerm;
    }

    public double filter(double raw) {
        double curved = withCurve(raw);
        double signal = smoothing * lastInput + (1 - smoothing) * curved;
        lastInput = signal;
        signal = withDead(signal);
        return signal;
    }
}
