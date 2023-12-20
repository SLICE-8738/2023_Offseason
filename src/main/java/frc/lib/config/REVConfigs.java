package frc.lib.config;

import com.revrobotics.CANSparkMax.IdleMode;

public final class REVConfigs {

    public class SparkMaxConfiguration {

        public IdleMode idleMode;
        public boolean inverted;

        public int statusFrame0RateMs = 10;
        public int statusFrame1RateMs;
        public int statusFrame2RateMs;

        public double openLoopRampRate;
        public double closedLoopRampRate;

        public boolean enableVoltageCompensation;
        public double nominalVoltage;

        public int currentLimit;    

        public SparkMaxConfiguration(
            IdleMode idleMode, 
            boolean inverted, 
            boolean enableVoltageCompensation, 
            double nominalVoltage, 
            double openLoopRampRate, 
            double closedLoopRampRate, 
            int currentLimit, 
            int statusFrame1RateMs,
            int statusFrame2RateMs) {

                this.idleMode = idleMode;
                this.inverted = inverted;
                this.enableVoltageCompensation = enableVoltageCompensation;
                this.nominalVoltage = nominalVoltage;
                this.openLoopRampRate = openLoopRampRate;
                this.closedLoopRampRate = closedLoopRampRate;
                this.currentLimit = currentLimit;
                this.statusFrame1RateMs = statusFrame1RateMs;
                this.statusFrame2RateMs = statusFrame2RateMs;

        }

    }

}
