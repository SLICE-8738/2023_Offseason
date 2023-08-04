// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  //Create 2 motors
  private CANSparkMax IntakeMotor;

  //constructor 
  public Intake() {
    //Constructs Intake Motor
    IntakeMotor = new CANSparkMax(5, MotorType.kBrushless); 
  }

  /** Makes motor spin to pick up cone */
  public void IntakeSpinUp(){
    IntakeMotor.set(0.4);
  }

  /** Makes motor spin to spit out cone*/
  public void IntakeSpinOut(){
    IntakeMotor.set(-0.4);
  }

  /** Makes motor stop spinning */
  private void IntakeStopSpinning(){
    IntakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
