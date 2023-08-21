// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoSelector;
import frc.robot.commands.LambdaCommand;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Drivetrain.sequences.TestTrajectoryFollowerSequence;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;

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

  // The robot's subsystems and commands are defined here...

  //Subsystems
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Limelight m_limelight = new Limelight();

  //Other Instantiated Classes
  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain);
  public final NodeSelector m_nodeSelector = new NodeSelector(m_drivetrain);
  public final ShuffleboardData m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_autoSelector);

  //Commands
  public final SwerveDriveCommand m_swerveDriveOpenLoop = new SwerveDriveCommand(m_drivetrain, driverController, true, true);
  public final SwerveDriveCommand m_swerveDriveClosedLoop = new SwerveDriveCommand(m_drivetrain, driverController, false, true);
  public final SetPercentOutputCommand m_setDrivePercentOutput = new SetPercentOutputCommand(m_drivetrain, 0.1, 0);
  public final ResetFieldOrientedHeading m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);
  public final SetInitialPositionCommand m_setInitialPosition = new SetInitialPositionCommand(m_drivetrain);
  public final LambdaCommand m_xControllerTestSequence = new LambdaCommand(() -> new TestTrajectoryFollowerSequence(m_drivetrain, new Transform2d(new Translation2d(-2, 0), new Rotation2d())));
  public final LambdaCommand m_yControllerTestSequence = new LambdaCommand(() -> new TestTrajectoryFollowerSequence(m_drivetrain, new Transform2d(new Translation2d(0, 2), new Rotation2d())));
  public final LambdaCommand m_thetaControllerTestSequence = new LambdaCommand(() -> new TestTrajectoryFollowerSequence(m_drivetrain, new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(270))));

  public final Command m_nodeAlignAndPosition = new LambdaCommand(m_nodeSelector::getNodeSequence).until(Button.nodeAlignAndPositionCancel);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    
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
    //Sets the current angle of the robot as an offset to field-oriented driving once pressed
    Button.resetFieldOrientedHeading.onTrue(m_resetFieldOrientedHeading);
    //Resets the robot's odometry to the starting position selected on Shuffleboard once pressed
    Button.setInitialPosition.onTrue(m_setInitialPosition);
    //Runs the robot along a straight trajecotry parallel to the X axis while pressed
    Button.xControllerTest.whileTrue(m_xControllerTestSequence);
    //Runs the robot along a straight trajectory parallel to the Y axis while pressed
    Button.yControllerTest.whileTrue(m_yControllerTestSequence);
    //Runs the robot along a trajectory rotating the robot 270 degrees while pressed
    Button.thetaControllerTest.whileTrue(m_thetaControllerTestSequence);
    //Aligns the robot with the node selected on Shuffleboard once pressed
    Button.nodeAlignAndPosition.onTrue(m_nodeAlignAndPosition);

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
