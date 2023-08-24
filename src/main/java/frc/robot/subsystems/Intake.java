// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.factories.SparkMaxFactory;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  //Create 2 motors
  private CANSparkMax IntakeMotor;

  //constructor 
  public Intake() {
    //Constructs Intake Motor
    IntakeMotor = SparkMaxFactory.createDefaultSparkMax(Constants.kIntake.MOTOR_ID); 
  }

  /** Makes motor spin to pick up cone */
  public void IntakeSpinUp(){
    IntakeMotor.set(0.4);
  }

  /** Makes motor spin to spit out cone*/
  public void IntakeSpinOut(){
    IntakeMotor.set(-0.4);
  }

  /** Makes motor spin at a slower rate for holding in place */
  public void IntakeSpinHoldUp() {
    IntakeMotor.set(0.2);
  }

  /** Makes motor stop spinning */
  public void IntakeStopSpinning(){
    IntakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
