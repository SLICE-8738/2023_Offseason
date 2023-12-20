package frc.lib.factories;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are nost set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    public static class Configuration {
        
        public NeutralModeValue NEUTRAL_MODE;
        // factory default
        public double NEUTRAL_DEADBAND;

        public boolean ENABLE_CURRENT_LIMIT;
        public double CURRENT_LIMIT;
        public double TRIGGER_THRESHOLD_CURRENT;
        public double TRIGGER_THRESHOLD_TIME;

        public InvertedValue INVERTED;
        public boolean SENSOR_PHASE;

        public double VELOCITY_UPDATE_FREQUENCY_HZ = 1000 / 45;
        public double POSITION_UPDATE_FREQUENCY_HZ = 4;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public Configuration(
                NeutralModeValue neutralMode, 
                double neutralDeadband, 
                double currentLimit, 
                double triggerThresholdCurrent, 
                boolean enableCurrentLimit, 
                boolean enableSoftLimit, 
                boolean enableLimitSwitch, 
                InvertedValue inverted, 
                boolean sensorPhase) {

            NEUTRAL_MODE = neutralMode;
            // factory default
            NEUTRAL_DEADBAND = neutralDeadband;

            CURRENT_LIMIT = currentLimit;
            TRIGGER_THRESHOLD_CURRENT = triggerThresholdCurrent;

            ENABLE_CURRENT_LIMIT = enableCurrentLimit;

            INVERTED = inverted;
            SENSOR_PHASE = sensorPhase;

        }

    }

    public static final Configuration kDefaultConfiguration = new Configuration(
        NeutralModeValue.Brake, 
        0.04, 
        48, 
        48,
        true,
        true,
        false,
        InvertedValue.CounterClockwise_Positive,
        false);

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id) {

        return createTalon(id, kDefaultConfiguration);

    }

    public static TalonFX createTalon(int id, Configuration config) {

        TalonFX talon = new TalonFX(id);
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        talonConfig.MotorOutput.Inverted = config.INVERTED;
        talonConfig.MotorOutput.NeutralMode = config.NEUTRAL_MODE;

        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.ENABLE_CURRENT_LIMIT;
        talonConfig.CurrentLimits.SupplyCurrentLimit = config.CURRENT_LIMIT;
        talonConfig.CurrentLimits.SupplyCurrentThreshold = config.TRIGGER_THRESHOLD_CURRENT;
        talonConfig.CurrentLimits.SupplyTimeThreshold = config.TRIGGER_THRESHOLD_TIME;

        talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.OPEN_LOOP_RAMP_RATE;
        talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.OPEN_LOOP_RAMP_RATE;

        talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = config.CLOSED_LOOP_RAMP_RATE;
        talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.CLOSED_LOOP_RAMP_RATE;

        talon.getConfigurator().apply(talonConfig);

        return talon;

    }

}