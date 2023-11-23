package frc.lib.factories;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private final static int kTimeoutMs = 100;

    public static class Configuration {
        
        public NeutralMode NEUTRAL_MODE;
        // factory default
        public double NEUTRAL_DEADBAND;

        public double CURRENT_LIMIT;
        public double TRIGGER_THRESHOLD_CURRENT;
        public double TRIGGER_THRESHOLD_TIME = 0.1;

        public boolean ENABLE_CURRENT_LIMIT;
        public boolean ENABLE_SOFT_LIMIT;
        public boolean ENABLE_LIMIT_SWITCH;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED;
        public boolean SENSOR_PHASE;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public Configuration(NeutralMode neutralMode, double neutralDeadband, double currentLimit, double triggerThresholdCurrent, boolean enableCurrentLimit, boolean enableSoftLimit, boolean enableLimitSwitch, boolean inverted, boolean sensorPhase) {

            NEUTRAL_MODE = neutralMode;
            // factory default
            NEUTRAL_DEADBAND = neutralDeadband;

            CURRENT_LIMIT = currentLimit;
            TRIGGER_THRESHOLD_CURRENT = triggerThresholdCurrent;

            ENABLE_CURRENT_LIMIT = enableCurrentLimit;
            ENABLE_SOFT_LIMIT = enableSoftLimit;
            ENABLE_LIMIT_SWITCH = enableLimitSwitch;

            INVERTED = inverted;
            SENSOR_PHASE = sensorPhase;

        }

    }

    private static final Configuration kDefaultConfiguration = new Configuration(
        NeutralMode.Brake, 
        0.04, 
        48, 
        48,
        true,
        true,
        false,
        false,
        false);

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id) {

        return createTalon(id, kDefaultConfiguration);

    }

    public static TalonFX createTalon(int id, Configuration config) {

        TalonFX talon = new TalonFX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);

        talon.configNominalOutputForward(0, kTimeoutMs);
        talon.configNominalOutputReverse(0, kTimeoutMs);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

        talon.configPeakOutputForward(1.0, kTimeoutMs);
        talon.configPeakOutputReverse(-1.0, kTimeoutMs);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                kTimeoutMs);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

        talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        talon.enableVoltageCompensation(false);

        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(config.ENABLE_CURRENT_LIMIT, config.CURRENT_LIMIT, config.TRIGGER_THRESHOLD_CURRENT, config.TRIGGER_THRESHOLD_TIME), kTimeoutMs);
        talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(config.ENABLE_CURRENT_LIMIT, config.CURRENT_LIMIT, config.TRIGGER_THRESHOLD_CURRENT, config.TRIGGER_THRESHOLD_TIME), kTimeoutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;

    }

}