// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
  //private DutyCycleEncoder armThroughboreEncoder, wristThroughboreEncoder;

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
    //wristMotorEncoder = wristMotor.getEncoder();
    //armMotorEncoder = armMotor.getAlternateEncoder(8192);
    wristMotorEncoder = wristMotor.getAlternateEncoder(8192);

    //defining PID motor Controllers
    armMotorController = armMotor.getPIDController();
    wristMotorController = wristMotor.getPIDController();

    //Setting the P, I(not), and D Gain for the lower motor
    armMotorController.setP(Constants.kArm.ELBOW_kP);
    armMotorController.setI(Constants.kArm.ELBOW_kI);
    armMotorController.setD(Constants.kArm.ELBOW_kD);
    //Setting the P, I(not), and D Gain for the higher motor
    wristMotorController.setP(Constants.kArm.WRIST_kP);
    wristMotorController.setI(Constants.kArm.WRIST_kI);
    wristMotorController.setD(Constants.kArm.WRIST_kD);

    armMotorEncoder.setPositionConversionFactor(Constants.kArm.ARM_POSITIONAL_CONVERSION_FACTOR);
    armMotorEncoder.setVelocityConversionFactor(Constants.kArm.ARM_VELOCITY_CONVERSION_FACTOR);
    armMotorEncoder.setPosition(Constants.kArm.STARTING_ELBOW_ANGLE);

    wristMotorEncoder.setPositionConversionFactor(Constants.kArm.WRIST_POSITIONAL_CONVERSION_FACTOR);
    wristMotorEncoder.setVelocityConversionFactor(Constants.kArm.WRIST_VELOCITY_CONVERSION_FACTOR);
    wristMotorEncoder.setPosition(Constants.kArm.STARTING_WRIST_ANGLE);

    /* This is code for using the throughbore encoders through the DIO ports rather than the Alternate Encoder ports on the SparkMaxes
    armThroughboreEncoder = new DutyCycleEncoder(Constants.kArm.ELBOW_THROUGHBORE_ID);
    wristThroughboreEncoder = new DutyCycleEncoder(Constants.kArm.WRIST_THROUGHBORE_ID);

    armThroughboreEncoder.setDistancePerRotation(Constants.kArm.ARM_DISTANCE_PER_ROTATION);
    armThroughboreEncoder.reset();

    wristThroughboreEncoder.setDistancePerRotation(Constants.kArm.WRIST_DISTANCE_PER_ROTATION); */



    targetElbowPosition = getElbowPosition();
    targetWristPosition = getWristPosition();

  }

  public void runElbow(double speed){
    armMotor.set(speed);
  }

  public void runWrist(double speed){
    wristMotor.set(speed);
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
    //return armThroughboreEncoder.getDistance() - Constants.kArm.STARTING_ELBOW_ANGLE;
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
    //return wristThroughboreEncoder.getDistance() - Constants.kArm.STARTING_WRIST_ANGLE;
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
