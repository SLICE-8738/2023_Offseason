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

public class Intake extends SubsystemBase {

  //initiating CANSparkMax motors
  private CANSparkMax lowerArmMotor;
  private CANSparkMax higherArmMotor;

  //initiating relative encoders for the CANSparkMaxs Encoders and PIDControllers
  private RelativeEncoder lowerMotorEncoder, higherMotorEncoder;

  private SparkMaxPIDController lowerMotorController, higherMotorController;


  /** Creates a new Intake. */
  public Intake() {
    
    //defining CANSparkMax Motors
    lowerArmMotor = new CANSparkMax(Constants.kArm.LOWER_MOTOR_ID, MotorType.kBrushless);
    higherArmMotor = new CANSparkMax(Constants.kArm.UPPER_MOTOR_ID, MotorType.kBrushless);

    //defining motor encoders
    lowerMotorEncoder = lowerArmMotor.getEncoder();
    higherMotorEncoder = higherArmMotor.getEncoder();

    //defining PID motor Controllers
    lowerMotorController = lowerArmMotor.getPIDController();
    higherMotorController = higherArmMotor.getPIDController();

    //Setting the P, I(not), and D Gain for the lower motor
    lowerMotorController.setP(0);
    lowerMotorController.setD(0);
    //Setting the P, I(not), and D Gain for the higher motor
    higherMotorController.setP(0);
    higherMotorController.setD(0);


  }

  /**
   * This method sets the reference for the motors PID Controller
   * @param value
   */
  public void setArmController(double value){
    lowerMotorController.setReference(value, ControlType.kPosition);
    higherMotorController.setReference(value, ControlType.kPosition);
    
  }

  /* This method will make sure the arm does not hit the elevator by preventing it from going past a certain position */
  public void armCheck(){
    //This method has yet to be implemented
  }

  /* This method will make sure the wrist does not extend backwards farther than it should by preventing it from going past a certain position */
  public void wristCheck(){
    //This method has yet to be implemented
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lowerMotorEncoder.getPosition();
    higherMotorEncoder.getPosition();
  }
}
