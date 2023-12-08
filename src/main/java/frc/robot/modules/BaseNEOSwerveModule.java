package frc.robot.modules;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.factories.SparkMaxFactory;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class BaseNEOSwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private SwerveModuleState targetState = new SwerveModuleState();
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.kDrivetrain.DRIVE_KS, Constants.kDrivetrain.DRIVE_KV, Constants.kDrivetrain.DRIVE_KA);

  private double simDistance = 0;

  public BaseNEOSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = SparkMaxFactory.createDefaultPositionSparkMax(moduleConstants.angleMotorID);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = SparkMaxFactory.createDefaultPrioritySparkMax(moduleConstants.driveMotorID);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    targetState = desiredState;

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void setPercentOutput(double drivePercentOutput, double anglePercentOutput) {

    driveMotor.set(drivePercentOutput);
    angleMotor.set(anglePercentOutput);

  }

  private Rotation2d waitForCANcoder(){
    /* wait for up to 250ms for a new CANcoder position */
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue());
  }

  public void resetToAbsolute() {
    double absolutePosition = waitForCANcoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(new CTREConfigs().swerveCANcoderConfig);
  }

  private void configAngleMotor() {
    integratedAngleEncoder.setPositionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR_DEGREES);
    angleController.setP(Constants.kDrivetrain.ANGLE_KP);
    angleController.setI(Constants.kDrivetrain.ANGLE_KI);
    angleController.setD(Constants.kDrivetrain.ANGLE_KD);
    angleController.setFF(Constants.kDrivetrain.ANGLE_KFF);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveEncoder.setVelocityConversionFactor(Constants.kDrivetrain.DRIVE_VELOCITY_CONVERSION_FACTOR);
    driveEncoder.setPositionConversionFactor(Constants.kDrivetrain.DRIVE_POSITION_CONVERSION_FACTOR);
    driveController.setP(Constants.kDrivetrain.DRIVE_KP);
    driveController.setI(Constants.kDrivetrain.DRIVE_KI);
    driveController.setD(Constants.kDrivetrain.DRIVE_KD);
    driveController.setFF(Constants.kDrivetrain.DRIVE_KFF);
    driveEncoder.setPosition(0.0);
  }

  public void setAnglePIDF(double kP, double kI, double kD, double kF) {
    angleController.setP(kP);
    angleController.setI(kI);
    angleController.setD(kD);
    angleController.setFF(kF);
  }

  public void setDrivePIDF(double kP, double kI, double kD, double kF) {
    driveController.setP(kP);
    driveController.setI(kI);
    driveController.setD(kD);
    driveController.setFF(kF);
  }

  public void setDriveIdleMode(boolean setBrakeMode) {

    if(setBrakeMode) {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }
    else {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

  }

  public void setAngleIdleMode(boolean setBrakeMode) {

    if(setBrakeMode) {
        angleMotor.setIdleMode(IdleMode.kBrake);
    }
    else {
        angleMotor.setIdleMode(IdleMode.kCoast);
    }

  }

  public void setMaxDriveOutput(double max) {

    driveController.setOutputRange(-max, max);

  }

  public void setMaxAngleOutput(double max) {

    angleController.setOutputRange(-max, max);

  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kDrivetrain.MAX_LINEAR_VELOCITY * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  public double getDriveMotorOutputCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public void setSimulationPosition() {
    simDistance += driveEncoder.getVelocity() * 0.02;
    driveEncoder.setPosition(simDistance);
    integratedAngleEncoder.setPosition(lastAngle.getDegrees());
  }
}