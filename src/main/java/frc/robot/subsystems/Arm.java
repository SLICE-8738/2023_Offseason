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

  }

  /**
   * This method sets the reference for arm's CanSparkMax PID Controller
   * @param value
   */
  public void setArmController(double value){
    armMotorController.setReference(value, ControlType.kPosition);
  }

  public void spinArmMotor(double value){
    armMotor.set(0.3);
  }
  /**
   * This method sets the reference for the wrist's CanSparkMax PID Controller 
   * @param value
   */
  public void setWristController(double value){
    wristMotorController.setReference(value, ControlType.kPosition);
  }

  

  /* This method will make sure the arm does not hit the elevator by preventing it from going past a certain position */
  public void armCheck(){
    //TODO: This method has yet to be implemented
  }

  /* This method will make sure the wrist does not extend backwards farther than it should by preventing it from going past a certain position */
  public void wristCheck(){
    //TODO: This method has yet to be implemented
  }

  public void wristSecureCube(){
    wristMotor.set(0.25);
  }

  /**
   * 
   * @return the position of the arm motor in rotations
   */
  public double armReturn(){
    return armMotorEncoder.getPosition();
  }

  public double getWristOutput() {
    return wristMotor.getOutputCurrent();
  }

  public double getArmOutput() {
    return armMotor.getOutputCurrent();
  }

  /**
   * 
   * @return the position of the wrist motor in rotations
   */
  public double wristReturn(){
    return wristMotorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
