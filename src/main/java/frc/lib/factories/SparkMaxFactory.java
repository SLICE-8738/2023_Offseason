package frc.lib.factories;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SparkMaxFactory {
    
    public static class Configuration {

        public IdleMode IDLE_MODE;
        public boolean INVERTED;

        public int STATUS_FRAME_0_RATE_MS = 10;
        public int STATUS_FRAME_1_RATE_MS;
        public int STATUS_FRAME_2_RATE_MS;

        public double OPEN_LOOP_RAMP_RATE;
        public double CLOSED_LOOP_RAMP_RATE;

        public boolean ENABLE_VOLTAGE_COMPENSATION;
        public double NOMINAL_VOLTAGE;

        public int CURRENT_LIMIT;

        public Configuration(
            IdleMode idleMode, 
            boolean inverted, 
            boolean enableVoltageCompensation, 
            double nominalVoltage, 
            double openLoopRampRate, 
            double closedLoopRampRate, 
            int currentLimit, 
            int status1FrameRate,
            int status2FrameRate) {

            IDLE_MODE = idleMode;
            INVERTED = inverted;

            OPEN_LOOP_RAMP_RATE = openLoopRampRate;
            CLOSED_LOOP_RAMP_RATE = closedLoopRampRate;

            ENABLE_VOLTAGE_COMPENSATION = enableVoltageCompensation;
            NOMINAL_VOLTAGE = nominalVoltage;

            CURRENT_LIMIT = currentLimit;

            STATUS_FRAME_1_RATE_MS = status1FrameRate;
            STATUS_FRAME_2_RATE_MS = status2FrameRate;

        }

    }

    private static final Configuration kDefaultConfiguration = new Configuration(
        IdleMode.kBrake,
        false,
        false,
        12,
        0.0,
        0.0,
        30,
        1000,
        1000);

    private static final Configuration kDefaultVelocityConfiguration = new Configuration(
        IdleMode.kBrake,
        Constants.kDrivetrain.DRIVE_INVERT,
        false,
        12,
        0.0,
        0.0,
        30,
        200,
        1000);

    private static final Configuration kDefaultPositionConfiguration = new Configuration(
        IdleMode.kBrake,
        Constants.kDrivetrain.ANGLE_INVERT,
        false,
        12,
        0.0,
        0.0,
        20,
        1500,
        200);

    /**
     * Creates a CANSparkMax object with a configuration optimized for less significant motors.
     */
    public static CANSparkMax createDefaultSparkMax(int id) {

        return createSparkMax(id, kDefaultConfiguration);

    }

    /**
     * Creates a CANSparkMax object with a configuration optimized for motors primarily used with
     * velocity control.
     */
    public static CANSparkMax createDefaultVelocitySparkMax(int id) {

        return createSparkMax(id, kDefaultVelocityConfiguration);

    }

    /**
     * Creates a CANSparkMax object with a configuration optimized for motors primarily used with
     * position control.
     */
    public static CANSparkMax createDefaultPositionSparkMax(int id) {

        return createSparkMax(id, kDefaultPositionConfiguration);
    
    }

    private static void handleREVLibError(int id, REVLibError error, String message) {

        if (error != REVLibError.kOk) {
            DriverStation.reportError(
                    "Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
        }

    }

    /** Creates a CANSparkMax object with specifiable configuration settings. */
    public static CANSparkMax createSparkMax(int id, Configuration config) {
        // Delay for CAN bus bandwidth to clear up.
        Timer.delay(0.25);
        CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
        handleREVLibError(id, sparkMax.setCANTimeout(200), "set timeout");

        sparkMax.restoreFactoryDefaults();

        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

        sparkMax.clearFaults();

        handleREVLibError(id, sparkMax.setIdleMode(config.IDLE_MODE), "set idle");
        sparkMax.setInverted(config.INVERTED);
        handleREVLibError(id, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
        handleREVLibError(id, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

        if (config.ENABLE_VOLTAGE_COMPENSATION) {
            handleREVLibError(id, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
        } 
        else {
            handleREVLibError(id, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        handleREVLibError(id, sparkMax.setSmartCurrentLimit(config.CURRENT_LIMIT), "current limit");

        return sparkMax;

    }

}
