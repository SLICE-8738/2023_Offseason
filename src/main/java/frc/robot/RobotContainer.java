// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoSelector;
import frc.robot.commands.GoToState;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Drivetrain.sequences.IntakeCommandsSequence;
import frc.robot.commands.Elevator.ManualElevator;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OutTakeCommand;
import frc.robot.commands.LEDs.CustomRainbowLEDs;
import frc.robot.commands.LEDs.FlashColorCommand;
import frc.robot.commands.LEDs.RainbowLEDs;
import frc.robot.commands.LEDs.VariableModeLEDs;
import frc.robot.commands.StowCommands.Stow;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.StowState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //private static Joystick leftJoystick = Button.leftJoystick;
  //private static Joystick rightJoystick = Button.rightJoystick;

  private static GenericHID driverController = Button.driverController;
  private static GenericHID operatorController = Button.operatorController;

  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Intake m_intake = new Intake();
  public final Arm m_arm = new Arm();
  public final Elevator m_elevator = new Elevator();
  public final LEDs m_leds = new LEDs();
  public final Limelight m_limelight = new Limelight(); 

  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain);
  
  public final ShuffleboardData m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_elevator, m_arm, m_intake, m_autoSelector);

  // ==========================
  // Commands
  // ==========================

  // Drivetrain
  public final SwerveDriveCommand m_swerveDriveOpenLoop = new SwerveDriveCommand(m_drivetrain, driverController, true, true, m_elevator);
  public final SwerveDriveCommand m_swerveDriveClosedLoop = new SwerveDriveCommand(m_drivetrain, driverController, false, true, m_elevator);
  public final SetPercentOutputCommand m_setDrivePercentOutput = new SetPercentOutputCommand(m_drivetrain, 0.1, 0);
  public final ResetFieldOrientedHeading m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);

  // Intake
  public final IntakeCommand m_IntakeCommand = new IntakeCommand(m_intake, m_arm);
  public final OutTakeCommand m_OutTakeCommand = new OutTakeCommand(m_intake);

  // Elevator
  public final ManualElevator m_manualElevator = new ManualElevator(m_elevator, operatorController);

  // Arm
  public final ManualArm m_manualArm = new ManualArm(m_arm, operatorController);

  // LEDs
  public final CustomRainbowLEDs m_cubeLights = new CustomRainbowLEDs(m_leds, 140);
  public final CustomRainbowLEDs m_coneLights = new CustomRainbowLEDs(m_leds, 28);
  public final CustomRainbowLEDs m_solidOrangeLEDs = new CustomRainbowLEDs(m_leds, 145);
  public final FlashColorCommand m_flashOrangeLEDs = new FlashColorCommand(m_leds, 175, 0.5, 0.5);
  public final RainbowLEDs m_rainbowLEDs = new RainbowLEDs(m_leds);
  public final VariableModeLEDs m_idleLEDs = new VariableModeLEDs(m_leds);

  // States
  public final Stow m_stow = new Stow(m_elevator, m_arm, m_intake);
  public final GoToState m_scoreHigh = new GoToState(m_elevator, m_arm, Constants.kRobotStates.highScore);
  public final GoToState m_scoreMid = new GoToState(m_elevator, m_arm, Constants.kRobotStates.midScore);
  public final GoToState m_scoreLow = new GoToState(m_elevator, m_arm, Constants.kRobotStates.lowScore);

  /* Intake Command Sequences */
  //Cubes
  public final IntakeCommandsSequence m_cubeDoubleSubstationSequence = 
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cube, Constants.kRobotStates.cubeDoubleSubstation);
  public final IntakeCommandsSequence m_cubeSingleSubstation =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cube, Constants.kRobotStates.cubeSingleSubstation);
  public final IntakeCommandsSequence m_cubeGround =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cube, Constants.kRobotStates.cubeGround);
    //Conditional Commands for the Cube. This conditional command executes a command based on whether the robot is facing the double substation or not
  public final ConditionalCommand m_cubeSubstationsConditionalCommand =
                new ConditionalCommand( m_cubeDoubleSubstationSequence, m_cubeSingleSubstation , () -> m_drivetrain.facingDoubleSub());
  
  //Cones
  public final IntakeCommandsSequence m_tippedConeDoubleSubstation =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cone , Constants.kRobotStates.tippedConeDoubleSubstation);
  public final IntakeCommandsSequence m_tippedConeGround =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cone, Constants.kRobotStates.tippedConeGround);
  public final IntakeCommandsSequence m_uprightConeDoubleSubstation =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cone, Constants.kRobotStates.uprightConeDoubleSubstation);
  public final IntakeCommandsSequence m_uprightConeGround =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cone, Constants.kRobotStates.uprightConeGround);
  public final IntakeCommandsSequence m_coneSingleSubstation =
                new IntakeCommandsSequence(m_intake, m_arm, m_elevator, StowState.Cone, Constants.kRobotStates.coneSingleSubstation);
    //Conditional Commands for the Cone. These conditonal commands are for determining which substation the robot is at, and whether the cone is tipped or upright.
  public final ConditionalCommand m_coneDoubleSubstationConditionalCommand =
                new ConditionalCommand(m_uprightConeDoubleSubstation, m_tippedConeDoubleSubstation , () -> Constants.kRobotStates.coneUpright);
  public final ConditionalCommand m_coneSubstationsConditionalCommand =
                new ConditionalCommand(m_coneDoubleSubstationConditionalCommand, m_coneSingleSubstation , () -> m_drivetrain.facingDoubleSub());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    m_elevator.setDefaultCommand(m_manualElevator);
    m_arm.setDefaultCommand(m_manualArm);
    m_leds.setDefaultCommand(m_idleLEDs);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Sets drivetrain drive motors at a fixed percent output while pressed
    Button.setDrivePercentOutput.whileTrue(m_setDrivePercentOutput);
    Button.resetFieldOrientedHeading.onTrue(m_resetFieldOrientedHeading);

    Button.outtake.whileTrue(m_OutTakeCommand);

    // Scoring
    Button.scoreHigh.onTrue(m_scoreHigh);
    Button.scoreMid.onTrue(m_scoreMid);
    Button.scoreLow.onTrue(m_scoreLow);

    //Intake Stow
    Button.cubeGroundIntake.onTrue(m_cubeGround);
    Button.tippedConeGroundIntake.onTrue(m_tippedConeGround);
    Button.uprightConeGroundIntake.onTrue(m_uprightConeGround);

    Button.cubeSubstationIntake.onTrue(m_cubeSubstationsConditionalCommand);
    Button.coneSubstationIntake.onTrue(m_coneSubstationsConditionalCommand);

    // Stow
    Button.stow.onTrue(m_stow);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.getAutoMode();
  }

}
