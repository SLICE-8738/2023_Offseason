/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.AutoSelector.StartingPosition;
import frc.robot.modules.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

  //private final SparkMaxSwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;
  private final BaseNEOSwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;

  private final SwerveDriveOdometry m_swerveDrivetrainOdometry;

  private final AHRS navXGyro;

  public final Field2d m_field2d;

  private final Timer autoTrajectoryTimer;

  private Trajectory currentAutoTrajectory;

  private Rotation2d fieldOrientedOffset;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    //The angle offset arguments for these swerve modules are placeholders for now
    /*leftModuleFront = new SparkMaxSwerveModule(Constants.kDrivetrain.Mod0.DRIVE_MOTOR_ID, Constants.kDrivetrain.Mod0.ANGLE_MOTOR_ID, Constants.kDrivetrain.Mod0.ANGLE_OFFSET.getRadians());
    leftModuleBack = new SparkMaxSwerveModule(Constants.kDrivetrain.Mod1.DRIVE_MOTOR_ID, Constants.kDrivetrain.Mod1.ANGLE_MOTOR_ID, Constants.kDrivetrain.Mod1.ANGLE_OFFSET.getRadians());
    rightModuleFront = new SparkMaxSwerveModule(Constants.kDrivetrain.Mod2.DRIVE_MOTOR_ID, Constants.kDrivetrain.Mod2.ANGLE_MOTOR_ID, Constants.kDrivetrain.Mod2.ANGLE_OFFSET.getRadians());
    rightModuleBack = new SparkMaxSwerveModule(Constants.kDrivetrain.Mod3.DRIVE_MOTOR_ID, Constants.kDrivetrain.Mod3.ANGLE_MOTOR_ID, Constants.kDrivetrain.Mod3.ANGLE_OFFSET.getRadians());*/

    leftModuleFront = new BaseNEOSwerveModule(0, Constants.kDrivetrain.Mod0.CONSTANTS);
    leftModuleBack = new BaseNEOSwerveModule(1, Constants.kDrivetrain.Mod1.CONSTANTS);
    rightModuleFront = new BaseNEOSwerveModule(2, Constants.kDrivetrain.Mod2.CONSTANTS);
    rightModuleBack = new BaseNEOSwerveModule(3, Constants.kDrivetrain.Mod3.CONSTANTS);

    navXGyro = new AHRS(Constants.kDrivetrain.NAVX_PORT);

    Timer.delay(1.0);
    resetModulesToAbsolute();
    resetHeading();

    m_field2d = new Field2d();

    autoTrajectoryTimer = new Timer();

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(m_field2d);

    m_swerveDrivetrainOdometry = new SwerveDriveOdometry(
      Constants.kDrivetrain.kSwerveKinematics, 
      getRotation2d(), 
      getPositions(),
      new Pose2d(8.28, 4, Rotation2d.fromDegrees(0)));

    fieldOrientedOffset = new Rotation2d();

  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run

    updateOdometry();

    m_field2d.setRobotPose(getPose());

    SmartDashboard.putNumber("Front Left Integrated Speed", leftModuleFront.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Integrated Speed", rightModuleFront.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Integrated Speed", leftModuleBack.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Integrated Speed", rightModuleBack.getState().speedMetersPerSecond);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets the idle mode of all drive motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * drive motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setDriveIdleMode(boolean enableBrakeMode) {

    leftModuleFront.setDriveIdleMode(enableBrakeMode);
    leftModuleBack.setDriveIdleMode(enableBrakeMode);
    rightModuleFront.setDriveIdleMode(enableBrakeMode);
    rightModuleBack.setDriveIdleMode(enableBrakeMode);

  }

  /**
   * Sets the idle mode of all angle motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * angle motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setAngleIdleMode(boolean enableBrakeMode) {

    leftModuleFront.setAngleIdleMode(enableBrakeMode);
    leftModuleBack.setAngleIdleMode(enableBrakeMode);
    rightModuleFront.setAngleIdleMode(enableBrakeMode);
    rightModuleBack.setAngleIdleMode(enableBrakeMode);

  }

  /**
   * Sets all drive motors to specified proportional, integral, derivative, and feedforward gains.
   * 
   * @param kP The desired proportional gain for all drive motors to be set to.
   * @param kI The desired integral gain for all drive motors to be set to.
   * @param kD The desired derivative gain for all drive motors to be set to.
   * @param kF The desired feedforward gain for all drive motors to be set to.
   */
  public void setDrivePIDF(double kP, double kI, double kD, double kF) {

    leftModuleFront.setDrivePIDF(kP, kI, kD, kF);
    leftModuleBack.setDrivePIDF(kP, kI, kD, kF);
    rightModuleFront.setDrivePIDF(kP, kI, kD, kF);
    rightModuleBack.setDrivePIDF(kP, kI, kD, kF);

  }

  /**
   * Sets all angle motors to specified proportional, integral, derivative, and feedforward gains.
   * 
   * @param kP The desired proportional gain for all angle motors to be set to.
   * @param kI The desired integral gain for all angle motors to be set to.
   * @param kD The desired derivative gain for all angle motors to be set to.
   * @param kF The desired feedforward gain for all drive motors to be set to.
   */
  public void setAnglePIDF(double kP, double kI, double kD, double kF) {

    leftModuleFront.setAnglePIDF(kP, kI, kD, kF);
    leftModuleBack.setAnglePIDF(kP, kI, kD, kF);
    rightModuleFront.setAnglePIDF(kP, kI, kD, kF);
    rightModuleBack.setAnglePIDF(kP, kI, kD, kF);

  }

  /**
   * Sets the maxiumum and reverse power of all native drive PIDF controllers to a specified value.
   * 
   * @param max The desired maximum forward and reverse power to set all native drive PIDF controllers to.
   */
  public void setMaxDriveOutput(double max) {

    leftModuleFront.setMaxDriveOutput(max);
    leftModuleBack.setMaxDriveOutput(max);
    rightModuleFront.setMaxDriveOutput(max);
    rightModuleBack.setMaxDriveOutput(max);

  }

  /**
   * Sets the maxiumum and reverse power of all native angle PID controllers to a specified value.
   * 
   * @param max The desired maximum forward and reverse power to set all native angle PID controllers to.
   */
  public void setMaxAngleOutput(double max) {

    leftModuleFront.setMaxAngleOutput(max);
    leftModuleBack.setMaxAngleOutput(max);
    rightModuleFront.setMaxAngleOutput(max);
    rightModuleBack.setMaxAngleOutput(max);

  }

  /**
   * Drives the robot at either given field-relative X, Y, and rotational velocities or given
   * robot-relative forward, sideways, and rotational velocities.
   * 
   * <p> If using robot-relative velocities, the X component of the Translation2d object should be the forward velocity
   * and the Y component should be the sideways velocity.
   * 
   * @param transform A Transform2d object representing either the desired field-relative velocities in meters/second for the 
   *                  robot to move at along the X and Y axes of the field(forwards/backwards from driver POV), or the desired robot-relative forward 
   *                  and sideways velocities in meters/second for the robot to move at, as well as the desired velocity in radians/second for the 
   *                  robot to rotate at.
   * 
   * @param isOpenLoop Whether the accordingly generated states for the given velocities should be set using open loop control for the drive motors
   *                   of the swerve modules.
   * @param isFieldRelative Whether the given velocities are relative to the field or not.
   */
  public void swerveDrive(Transform2d transform, boolean isOpenLoop, boolean isFieldRelative) {

    Rotation2d rotationWithOffset = getRotation2d().minus(fieldOrientedOffset);
    if (rotationWithOffset.getDegrees() > 360) {
      rotationWithOffset.minus(Rotation2d.fromDegrees(360));
    }
    if (rotationWithOffset.getDegrees() < 0) {
      rotationWithOffset.plus(Rotation2d.fromDegrees(360));
    }

    SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(
      isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(
      transform.getX(), 
      transform.getY(), 
      transform.getRotation().getRadians(), 
      rotationWithOffset) 
      : new ChassisSpeeds(transform.getX(), transform.getY(), transform.getRotation().getRadians()));

      
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    leftModuleFront.setDesiredState(states[0], isOpenLoop);
    leftModuleBack.setDesiredState(states[1], isOpenLoop);
    rightModuleFront.setDesiredState(states[2], isOpenLoop);
    rightModuleBack.setDesiredState(states[3], isOpenLoop);

  }

  /**
   * Sends the poses of a desired trajectory to the Field2d object.
   * 
   * @param trajectory The desired trajectory to send to the Field2d object.
   */
  public void setField2d(Trajectory trajectory) {

    // Pushes the trajectory to Field2d.
    m_field2d.getObject("Trajectory").setTrajectory(trajectory);

  }

  /**
   * Resets and starts a timer in order to provide a time 
   * since the beginning of the trajectory to sample from.
   */
  public void startAutoTrajectoryTimer() {

    autoTrajectoryTimer.reset();
    autoTrajectoryTimer.start();

  }

  /**
   * Sets the auto trajectory used to sample the state at each time step from.
   * 
   * @param trajectory The current auto trajectory to sample from.
   */
  public void setCurrentAutoTrajectory(Trajectory trajectory) {

    currentAutoTrajectory = trajectory;
    
  }

  /**
   * Samples and obtains the rotation at the current time step of the current auto trajectory. 
   * 
   * @return The rotation of the robot of at the current time step of the current auto trajectory.
   */
  public Rotation2d getAutoTrajectoryRotation() {

    return currentAutoTrajectory.sample(autoTrajectoryTimer.get()).poseMeters.getRotation();

  }

  /**
   * Updates the drivetrain odometry object to the robot's current position on the field.
   * 
   * @return The new updated pose of the robot.
   */
  public Pose2d updateOdometry() {

    return m_swerveDrivetrainOdometry.update(getRotation2d(), getPositions());

  }

  /**
   * Returns the current pose of the robot without updating
   * the odometry.
   * 
   * @return The current estimated pose of the robot.
   */
  public Pose2d getPose() {
    return m_swerveDrivetrainOdometry.getPoseMeters();
  }

  /**
   * Obtains and returns the current positions of all drivetrain swerve modules.
   * 
   * @return The current positions of all drivetrain swerve modules.
   */
  public SwerveModulePosition[] getPositions() {

    SwerveModulePosition[] positions = {
      leftModuleFront.getPosition(),
      leftModuleBack.getPosition(),
      rightModuleFront.getPosition(),
      rightModuleBack.getPosition()};

    return positions;

  }

  /**
   * Obtains and returns the current states of all drivetrain swerve modules.
   * 
   * @return The current states of all drivetrain swerve modules.
   */
  public SwerveModuleState[] getStates() {

    SwerveModuleState[] states = {
      leftModuleFront.getState(),
      leftModuleBack.getState(),
      rightModuleFront.getState(),
      rightModuleBack.getState()};

      return states;

  }

  /**
   * Obtains and returns the target states that the drivetrain swerve modules have been set to.
   * 
   * @return The target states that the drivetrain swerve modules have been set to.
   */
  public SwerveModuleState[] getTargetStates() {

    SwerveModuleState[] targetStates = {
      leftModuleFront.getTargetState(),
      leftModuleBack.getTargetState(),
      rightModuleFront.getTargetState(),
      rightModuleBack.getTargetState()};

    return targetStates;
    
  }

  /**
   * Obtains and returns the current absolute angle readings
   * from the CANCoders of all swerve modules without offsets.
   * 
   * @return The current absolute angle readings from the CANCoders
   *         of all swerve modules without offsets.
   */
  public double[] getCANCoderAngles() {

    double[] angles = {
      leftModuleFront.getCanCoder().getDegrees(),
      leftModuleBack.getCanCoder().getDegrees(),
      rightModuleFront.getCanCoder().getDegrees(),
      rightModuleBack.getCanCoder().getDegrees()};

    return angles;

  }

  /**
   * Sets the positions of the integrated angle motor
   * encoders of all swerve modules to the absolute position
   * readings of the CANCoders with their offsets being taken
   * into account.
   */
  public void resetModulesToAbsolute() {

    leftModuleFront.resetToAbsolute();
    leftModuleBack.resetToAbsolute();
    rightModuleFront.resetToAbsolute();
    rightModuleBack.resetToAbsolute();

  }

  /**
   * Resets the position of the odometry object using a specified position.
   * 
   * @param position The desired position to reset the odometry of the robot to.
   */
  public void resetOdometry(Pose2d position) {

    m_swerveDrivetrainOdometry.resetPosition(getRotation2d(), getPositions(), position);

  }

  /**
   * Resets the position of the odometry object to the robot's initial position based
   * on the selected starting position on Shuffleboard.
   */
  public void setInitialPosition() {

    StartingPosition startingPosition = AutoSelector.getStoredStartingPosition();

    switch(startingPosition) {

      case BLUE_COMMUNITY_LEFT:
        resetOdometry(new Pose2d(1.83, 4.39, Rotation2d.fromDegrees(180)));
        break;
      case BLUE_COMMUNITY_MIDDLE:
        resetOdometry(new Pose2d(1.83, 2.71, Rotation2d.fromDegrees(180)));
        break;
      case BLUE_COMMUNITY_RIGHT:
        resetOdometry(new Pose2d(1.83, 1.07, Rotation2d.fromDegrees(180)));
        break;
      case RED_COMMUNITY_LEFT:
        resetOdometry(new Pose2d(14.69, 1.07, Rotation2d.fromDegrees(0)));
        break;
      case RED_COMMUNITY_MIDDLE:
        resetOdometry(new Pose2d(14.69, 2.71, Rotation2d.fromDegrees(0)));
        break;
      case RED_COMMUNITY_RIGHT:
        resetOdometry(new Pose2d(14.69, 4.39, Rotation2d.fromDegrees(0)));
        break;
      default:
        break;

    }

  }

  /**
   * 
   * @return
   */
  public void resetFieldOrientedHeading() {
    double error = getHeading() - 180;
    fieldOrientedOffset = Rotation2d.fromDegrees(error);
  }

  /**
   * Obtains and returns the current heading of the robot as a Rotation2d from the gyro object.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {

    return Rotation2d.fromDegrees(getHeading());
    
  }
  
  /**
   * Obtains and returns the current heading of the robot going positive counter-clockwise from 0 to 360 degrees from the gyro object.
   *
   * @return The current heading of the robot going counter-clockwise positive from 0 to 360 degrees.
   */
  public double getHeading() {

    return Constants.kDrivetrain.INVERT_GYRO? -navXGyro.getYaw() + 180 : navXGyro.getYaw() + 180;

  }

  public boolean facingDoubleSub() {
    double degrees = getPose().getRotation().getDegrees();
    return degrees > 0 && degrees < 135 || degrees > 315;
  }

  /**
   * Obtains and returns the current pitch of the robot from -180 to 180 degrees, with an offset of 1 degree from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an offset of 1 degree.
   */
  public double getPitch() {

    return navXGyro.getPitch() + 1;

  }

  /**
   * Obtains and returns the current roll of the robot from -180 to 180 degrees, with an offset of 1.7 degrees from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an offset of 1.7 degrees.
   */
  public double getRoll() {

    return navXGyro.getRoll() + 1.7;

  }

  /**
   * Resets the gyro yaw axis to a heading of 0.
   */
  public void resetHeading() {

    navXGyro.reset();

  }

  /**
   * Sets the desired states of all drivetrain swerve modules to a specified arrary of states using
   * closed loop control for the drive motors of the swerve modules.
   * 
   * @param states The desired states for all drivetrain swerve modules to be set to.
   */
  public void setModuleStates(SwerveModuleState[] states) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    leftModuleFront.setDesiredState(states[0], false);
    leftModuleBack.setDesiredState(states[1], false);
    rightModuleFront.setDesiredState(states[2], false);
    rightModuleBack.setDesiredState(states[3], false);

  }

  /**
   * Sets the drive and angle motors of all swerve modules to given drive and angle motor
   * percent outputs.
   * 
   * @param drivePercentOutput The percent output between -1 and 1 to set all drive motors to.
   * @param anglePercentOutput The percent output between -1 and 1 to set all angle motors to.
   */
  public void setPercentOutput(double drivePercentOutput, double anglePercentOutput) {

    leftModuleFront.setPercentOutput(drivePercentOutput, anglePercentOutput);
    leftModuleBack.setPercentOutput(drivePercentOutput, anglePercentOutput);
    rightModuleFront.setPercentOutput(drivePercentOutput, anglePercentOutput);
    rightModuleBack.setPercentOutput(drivePercentOutput, anglePercentOutput);

  }

  /**
   * Sets all drivetrain swerve modules to states with speeds of 0 and the current angles of the modules.
   */
  public void stopDrive() {

    SwerveModuleState[] stopStates = {
      new SwerveModuleState(0, leftModuleFront.getState().angle),
      new SwerveModuleState(0, leftModuleBack.getState().angle),
      new SwerveModuleState(0, rightModuleFront.getState().angle),
      new SwerveModuleState(0, rightModuleBack.getState().angle)};


    setModuleStates(stopStates);
    
  }

}
