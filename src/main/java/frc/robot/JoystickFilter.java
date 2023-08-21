// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;

/**
 * A class to filter the input of a joystick axis.
 */
public class JoystickFilter {
    private double lastInput;
    private final double deadzone, smoothing;
    private final boolean curve;

    private final double exponent, exponentPercent;
    /**
     * @param deadzone input from the joystick that is less than the deadzone will be ignored. -1 to 1
     * @param smoothing a higher value will smooth the input more but also increase input delay. 0 to 1
     */
    public JoystickFilter(double deadzone, double smoothing) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        this.exponent = 1;
        this.exponentPercent = 0;
        curve = false;
        
        lastInput = 0;
    }

    public JoystickFilter(double deadzone, double smoothing, double exponent, double exponentPercent) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        this.exponent = exponent;
        this.exponentPercent = exponentPercent;
        this.curve = true;
        
        lastInput = 0;
    }

    public double withDead(double raw) {
        return MathUtil.applyDeadband(raw, deadzone);
    }

    public double withCurve(double raw) {
        double firstTerm = exponentPercent * Math.pow(Math.abs(raw), exponent);
        firstTerm = Math.copySign(firstTerm, raw);
        double secondTerm = (1 - exponentPercent) * raw;
        return firstTerm + secondTerm;
    }

    public double filter(double raw) {
        double filtered = raw;
        if (curve) {
            filtered = withCurve(filtered);
        }
        filtered = smoothing * lastInput + (1 - smoothing) * filtered;
        double signal = withDead(filtered);
        lastInput = signal;
        return signal;
    }
}
