// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;

import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class kJoysticks {

    public static final int JOYSTICK_PORT = 1;

    public static final double driveExponent = 1.0;
    public static final double driveExponentPercent = 1;

    public static final double turnExponent = 1.0;
    public static final double turnExponentPercent = 1;

  }

  public final class kController {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 2;

  }

  public static final class kRobotStates {
    // TODO: Fill in these values
    public static final RobotState stow = new RobotState(0, 0, 28); // Found

    public static final RobotState highScore = new RobotState(4.88, 52.57, 17); // Found
    public static final RobotState midScore = new RobotState(2.782, 39.4, 20.48); // Found
    public static final RobotState lowScore = new RobotState(0, 0, 0);

    public static final RobotState cubeDoubleSubstation = new RobotState(4.25, 43.21, -119.78);
    public static final RobotState cubeSingleSubstation = new RobotState(0, 0, 0);
    public static final RobotState cubeGround = new RobotState(0, 86.78, -97.31); // Found

    public static final RobotState tippedConeDoubleSubstation = new RobotState(0, 0, 0);
    public static final RobotState tippedConeGround = new RobotState(0.64, 131.85, -97.85); // Found
    public static final RobotState uprightConeDoubleSubstation = new RobotState(3.7, 18.21, 55.41);
    public static final RobotState uprightConeGround = new RobotState(0, 78.4, -17.18); // Found
    public static final RobotState coneSingleSubstation = new RobotState(0.73, 14.85, -69.17);

    // Determines whether to use upright or tipped cone for double substation cone
    // intake
    public static final boolean coneUpright = true;

  }

  public static final class kDrivetrain {

    public static final Port NAVX_PORT = Port.kUSB;
    public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW- (except for some godforsaken
                                                     // reason CCW- CW+ is the one that actually works so ignore this)

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(22);
    public static final double WHEEL_BASE = Units.inchesToMeters(27);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1
    public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0); // (150/7):1

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Swerve Voltage Compensation */
    public static final double MAX_VOLTAGE = 12.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.01;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.001;
    public static final double ANGLE_KFF = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.075;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_KS = 0.23065;
    public static final double DRIVE_KV = 2.717;
    public static final double DRIVE_KA = 0.32115;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
    public static final double ANGLE_POSITION_CONVERSION_FACTOR_DEGREES = 360.0 / ANGLE_GEAR_RATIO;
    public static final double ANGLE_POSITION_CONVERSION_FACTOR_RADIANS = Math.PI * 2;
    public static final double ANGLE_VELOCITY_CONVERSION_FACTOR_DEGREES = ANGLE_POSITION_CONVERSION_FACTOR_DEGREES
        / 60.0;
    public static final double ANGLE_VELOCITY_CONVERSION_FACTOR_RADIANS = ANGLE_POSITION_CONVERSION_FACTOR_RADIANS
        / 60.0;

    /* Swerve Profiling Values */
    public static final double MAX_LINEAR_VELOCITY = 4.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 7; // radians per second

    /* Motor Inverts */
    public static final boolean DRIVE_INVERT = false;
    public static final boolean ANGLE_INVERT = true;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = false;

    // Charge Station Board and Balance Speeds (-1 - 1)
    // public static final double CHARGE_STATION_BALANCE_SPEED = 0.3;
    // public static final double BOARD_CHARGE_SPEED = -0.55;

    /*
     * Charge Station Board and Balance Speeds (meters/second)
     * THESE SPEEDS WERE CALCULATED BASED ON THE CURRENT MAXIMUM VELOCITY
     * CONSTANT AND ARE PLACEHOLDERS FOR NOW
     */
    public static final double CHARGE_STATION_BALANCE_SPEED = 1.35;
    public static final double BOARD_CHARGE_SPEED = 2.475;

    // The minimum angle the drivetrain must be at to stop when boarding the charge
    // station
    public static final double BOARD_CHARGE_MINIMUM_STOP_ANGLE = 6;
    // The amount the angle should drop below the maximum angle to stop boarding the
    // charge station and begin balancing
    public static final double BOARD_CHARGE_ANGLE_CHANGE_THRESHOLD = 3.8;

    // Autnomous Tolerances
    public static final double AUTO_DISTANCE_ERROR_TOLERANCE = 0.35;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 11;
      public static final int ANGLE_MOTOR_ID = 15;
      public static final int CANCODER_ID = 21;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(118.65);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int DRIVE_MOTOR_ID = 16;
      public static final int ANGLE_MOTOR_ID = 12;
      public static final int CANCODER_ID = 20;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(220.96);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 2 */
    public static final class Mod2 {
      public static final int DRIVE_MOTOR_ID = 17;
      public static final int ANGLE_MOTOR_ID = 13;
      public static final int CANCODER_ID = 22;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(156.6);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int DRIVE_MOTOR_ID = 14;
      public static final int ANGLE_MOTOR_ID = 18;
      public static final int CANCODER_ID = 23;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(301.64);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
          CANCODER_ID, ANGLE_OFFSET);
    }
  }

  public static final class kAutonomous {
    // THIS MAXIMUM LINEAR VELOCITY AND ACCELERATION ARE PLACEHOLDERS FOR NOW
    public static final double kMaxVelocityMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
    public static final double kMaxAngularVelocityRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    // THESE P GAINS ARE PLACEHOLDERS FOR NOW
    public static final double kPXController = 2.5;
    public static final double kPYController = 2.5;
    public static final double kPThetaController = 2.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularVelocityRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

  }

  public static final class kArm {
    public static final int ARM_MOTOR_ID = 3;
    public static final int WRIST_MOTOR_ID = 10;

    public static final double CUBE_THRESHOLD = 8; // TODO: get the threshold value

    public static final double ARM_GEAR_RATIO = 1.0 / 36.0;
    public static final double ARM_SPROCKET_RATIO = 18.0 / 60.0;
    public static final double ARM_POSITIONAL_CONVERSION_FACTOR = ARM_GEAR_RATIO * ARM_SPROCKET_RATIO * 360.0;
    public static final double ARM_VELOCITY_CONVERSION_FACTOR = ARM_POSITIONAL_CONVERSION_FACTOR / 60.0;

    // public static final double ARM_DISTANCE_PER_ROTATION = ARM_SPROCKET_RATIO *
    // 360.0;

    public static final double WRIST_GEAR_RATIO = 1.0 / 15.0;
    public static final double WRIST_SPROCKET_RATIO = 18.0 / 38.0;
    public static final double WRIST_POSITIONAL_CONVERSION_FACTOR = WRIST_GEAR_RATIO * WRIST_SPROCKET_RATIO * 360.0;
    public static final double WRIST_VELOCITY_CONVERSION_FACTOR = WRIST_POSITIONAL_CONVERSION_FACTOR / 60.0;

    // public static final double WRIST_DISTANCE_PER_ROTATION = WRIST_SPROCKET_RATIO
    // * 360.0;

    public static final double UPPER_ARM_LENGTH = 0.394; // Length of upper arm in meters, 15.5 inches
    public static final double LOWER_ARM_LENGTH = 0.584; // Length of lower arm in meters, 23 inches
    public static final double WRIST_LENGTH = 0.15367; // Length of wrist in meters, 6.05 inches

    public static final double ELBOW_POSITION_ERROR_TOLERANCE = 7.5;
    public static final double WRIST_POSITION_ERROR_TOLERANCE = 20;

    public static final double ELBOW_kP = 0.016;
    public static final double ELBOW_kI = 0.000001;
    public static final double ELBOW_kD = 0.0016;

    public static final double WRIST_kP = 12.0; // TODO: tune the PID values
    public static final double WRIST_kI = 0.0;
    public static final double WRIST_kD = 0.0;

    public static final double STARTING_ELBOW_ANGLE = 0.0;
    public static final double STARTING_WRIST_ANGLE = 28.0;

    public static final int ELBOW_THROUGHBORE_ID = 0; // TODO: get the elbow throughbore ID
    public static final int WRIST_THROUGHBORE_ID = 0; // TODO: get the wrist throughbore ID
  }

  public static final class kIntake {
    public static final int MOTOR_ID = 4;

    public static final double CONE_THRESHOLD = 120; // TODO: get the threshold value
  }

  public static final class kElevator {
    public static final double ELEVATOR_GEAR_RATIO = 1.0 / 25.0;
    public static final double ELEVATOR_SPROCKET_PITCH_DIAMETER = 0.0462;
    public static final double ELEVATOR_POSITIONAL_CONVERSION_FACTOR = ELEVATOR_GEAR_RATIO
        * ELEVATOR_SPROCKET_PITCH_DIAMETER * Math.PI * 8;
    public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = ELEVATOR_POSITIONAL_CONVERSION_FACTOR / 60.0;

    public static final double ELEVATOR_POSITION_ERROR_TOLERANCE = 0.05;

    public static final double MAX_ELEVATOR_HEIGHT = 5.0;

    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR_ID = 2;

    public static final double kP = 4.0; // TODO: tune the PID values
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final int LIMIT_SWITCH_1_CHANNEL = 9;
    public static final int LIMIT_SWITCH_2_CHANNEL = 8;

  }

  public static final class kLEDs {
    public static final int LED_PWM_PORT = 9;
    public static final int LED_LENGTH = 77;
  }
}