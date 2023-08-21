// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public enum StowState {
    Cone, Cube, Nothing
  };

  //initiating CANSparkMax motors
  private CANSparkMax armMotor;
  private CANSparkMax wristMotor;

  //initiating relative encoders for the CANSparkMaxs Encoders and PIDControllers
  private RelativeEncoder armMotorEncoder, wristMotorEncoder;

  private SparkMaxPIDController armMotorController, wristMotorController;
  public static StowState stowState;

  private double targetElbowPosition, targetWristPosition;


  /** Creates a new Arm Subsytem. */
  public Arm() {
    stowState = StowState.Nothing;
    //defining CANSparkMax Motors
    armMotor = new CANSparkMax(Constants.kArm.ARM_MOTOR_ID, MotorType.kBrushless);
    wristMotor = new CANSparkMax(Constants.kArm.WRIST_MOTOR_ID, MotorType.kBrushless);

    //defining motor encoders
    armMotorEncoder = armMotor.getEncoder();
    wristMotorEncoder = wristMotor.getEncoder();

    //defining PID motor Controllers
    armMotorController = armMotor.getPIDController();
    wristMotorController = wristMotor.getPIDController();

    //Setting the P, I(not), and D Gain for the lower motor
    armMotorController.setP(0);
    armMotorController.setI(0);
    armMotorController.setD(0);
    //Setting the P, I(not), and D Gain for the higher motor
    wristMotorController.setP(0);
    wristMotorController.setI(0);
    wristMotorController.setD(0);

    armMotorEncoder.setPositionConversionFactor(Constants.kArm.ARM_POSITIONAL_CONVERSION_FACTOR);
    armMotorEncoder.setVelocityConversionFactor(Constants.kArm.ARM_VELOCITY_CONVERSION_FACTOR);

    wristMotorEncoder.setPositionConversionFactor(Constants.kArm.WRIST_POSITIONAL_CONVERSION_FACTOR);
    wristMotorEncoder.setVelocityConversionFactor(Constants.kArm.WRIST_VELOCITY_CONVERSION_FACTOR);

    targetElbowPosition = getElbowPosition();
    targetWristPosition = getWristPosition();

  }

  /**
   * This method sets the reference for arm's CanSparkMax PID Controller
   * @param value
   */
  public void setArmController(double value){
    armMotorController.setReference(value, ControlType.kPosition);
    targetElbowPosition = value;
  }

  /**
   * This method sets the reference for the wrist's CanSparkMax PID Controller 
   * @param value
   */
  public void setWristController(double value){
    wristMotorController.setReference(value, ControlType.kPosition);
    targetWristPosition = value;
  }

  public void wristSecureCube(){
    wristMotor.set(0.25);
  }

  public void wristReleaseCube(){
    wristMotor.set(0);
  }

  /**
   * 
   * @return the position of the arm motor in rotations
   */
  public double getElbowPosition(){
    return armMotorEncoder.getPosition();
  }

  /**
   * 
   * @return Returns output current from wrist motor.
   */
  public double getWristOutput() {
    return wristMotor.getOutputCurrent();
  }

  /**
   * 
   * @return Returns output current from arm motor.
   */
  public double getArmOutput() {
    return armMotor.getOutputCurrent();
  }

  /**
   * 
   * @return the position of the wrist motor in rotations
   */
  public double getWristPosition(){
    return wristMotorEncoder.getPosition();
  }

  public boolean isElbowAtTarget(){
    return Math.abs(getElbowPosition() - targetElbowPosition) < Constants.kArm.ELBOW_POSITION_ERROR_TOLERANCE;
  }

  public boolean isWristAtTarget(){
    return Math.abs(getWristPosition() - targetWristPosition) < Constants.kArm.WRIST_POSITION_ERROR_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
