// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator2. */
  private CANSparkMax motorLeft, motorRight;
  private DigitalInput limitySwitchy1, limitySwitchy2;
  private SparkMaxPIDController PIDLeft, PIDRight;
  private RelativeEncoder encoderLeft, encoderRight;
  private boolean lock = true;
  public Elevator() {
    motorLeft = new CANSparkMax(1, MotorType.kBrushless);
    motorRight = new CANSparkMax(2, MotorType.kBrushless);
    limitySwitchy1 = new DigitalInput(9);
    limitySwitchy2 = new DigitalInput(8);
    PIDLeft = motorLeft.getPIDController();
    PIDRight = motorRight.getPIDController();
    encoderLeft = motorLeft.getEncoder();
    encoderRight = motorRight.getEncoder();


    PIDLeft.setP(0);
    PIDRight.setP(0);
    PIDLeft.setI(0);
    PIDRight.setI(0);
    PIDLeft.setD(0);
    PIDRight.setD(0);
  }
    
  public void setPIDController(double positionsetament) {
    motorLeft.set(1);                                                            // Reset speed to zoomy mode
    motorRight.set(1);
    lock = true;

    if ((limitySwitchy1.get()) || (limitySwitchy2.get())) {                            // If either limit switch is active
      if (positionsetament>(encoderLeft.getPosition()*encoderRight.getPosition())) {   // If desired set position is greater than average current position, then move
        lock = false;
        PIDLeft.setReference(positionsetament, ControlType.kPosition);
        PIDRight.setReference(-positionsetament, ControlType.kPosition);
      } 
    }
    PIDLeft.setReference(positionsetament, ControlType.kPosition);                    // If limit switches aren't active, then move
    PIDRight.setReference(-positionsetament, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limitySwitchy1.get();
    limitySwitchy2.get();
    if ((limitySwitchy1.get()) || (limitySwitchy2.get())) {                          // Set speed to 0 when the limit switches become active
      if (lock = true) {
        motorLeft.set(0);
        motorRight.set(0);}
      
    }
  }
}
