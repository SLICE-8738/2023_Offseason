// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.factories.SparkMaxFactory;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  //Create 2 motors
  private CANSparkMax IntakeMotor;

  private final ShuffleboardTab driverTab;

  private boolean gamePieceSecured = false;

  //constructor 
  public Intake() {
    //Constructs Intake Motor
    IntakeMotor = SparkMaxFactory.createDefaultVelocitySparkMax(Constants.kIntake.MOTOR_ID); 

    driverTab = Shuffleboard.getTab("Driver Tab");

    driverTab.addBoolean("Game Piece Secured", () -> gamePieceSecured).
    withPosition(4, 3).
    withSize(2, 1);
  }

  /** Makes motor spin to pick up cone */
  public void IntakeSpinUp(){
    IntakeMotor.set(-0.8);
  }

  /** Makes motor spin to spit out cone*/
  public void IntakeSpinOut(){
    IntakeMotor.set(0.8);
  }

  public void IntakeSpinUpCube() {
    IntakeMotor.set(-0.4);
  }

  /** Makes motor spin at a slower rate for holding in place */
  public void IntakeSpinHoldUp() {
    IntakeMotor.set(0.2);
  }

  public void IntakeRetainCube() {
    IntakeMotor.set(-0.05);
  }

  /** Makes motor stop spinning */
  public void IntakeStopSpinning(){
    IntakeMotor.set(0.0);
  }

  public double getOutputCurrent() {
    return IntakeMotor.getOutputCurrent();
  }

  public void setGamePieceSecured(boolean gamePieceSecured) {
    this.gamePieceSecured = gamePieceSecured;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
