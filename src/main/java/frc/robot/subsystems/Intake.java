// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

public class Intake extends SubsystemBase {
 
  private final CANSparkMax mandibleMotor, rotateMotor;

  private final RelativeEncoder mandibleEncoder, rotateEncoder;

  private final SparkMaxPIDController mandiblePID/*, rotatePID*/;

  /** Creates a new Elevator. */
  public Intake() {

    mandibleMotor = SparkMaxFactory.createDefaultSparkMax(Constants.intake_PIVOT_PORT);
    rotateMotor = SparkMaxFactory.createDefaultSparkMax(Constants.intake_MANDIBLE_PORT);

    mandibleEncoder = mandibleMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);
    rotateEncoder = rotateMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);

    mandiblePID = mandibleMotor.getPIDController();
    //rotatePID = rotateMotor.getPIDController();

  }

  public void setMandiblePID(double kP, double kI, double kD) {
    mandiblePID.setP(kP);
    mandiblePID.setI(kI);
    mandiblePID.setD(kD);
  } 

  /**
   * Opens the mandibles
   */
  public void openMandibles() {
    mandiblePID.setReference(Constants.intake_MANDIBLE_OPEN_POSITION, ControlType.kPosition);
  }

  /**
   * Close the mandibles
   */
  public void closeMandibles() {
    mandiblePID.setReference(Constants.intake_MANDIBLE_CLOSED_POSITION, ControlType.kPosition);
  }

  /**
   * Manually spin the mandible motor at a given speed
   * @param speed the speed the mandible motor spins at, from 1 to -1
   */
  public void runMandibles(double speed) {
    mandibleMotor.set(speed);
  }

  /**
   * Spins the wheels along the intake
   * @param speed the speed the wheels spin at, from 1 to -1. If positive, intake. if negative, eject.
   */
  public void runIntake(double speed) {
    // SKELETON CODE NOTE: might need to reverse speed so positive is intake and negative is eject
    rotateMotor.set(speed);
  }

  /**
   * @return the angular position of the mandibles, in [UNITS GO HERE]
   */
  public double getMandibleEncoderPosition() {

    return mandibleEncoder.getPosition();

  }

  /**
   * Resets the position reported by the encoder of the mandibles to the desired position
   * @param position the position the encoder will be set to, in rotations
   */
  public void setMandibleEncoderPosition(double position) {
    mandibleEncoder.setPosition(position);
  }

  /**
   * @return the angular position of the intake wheels, in [UNITS GO HERE]
   */
  public double getRotateEncoderPosition() {

    return rotateEncoder.getPosition();

  }

  public boolean mandibleVoltageSpike() {
    return mandibleMotor.getOutputCurrent() > Constants.intake_CALIBRATION_CURRENT_THRESHOLD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
