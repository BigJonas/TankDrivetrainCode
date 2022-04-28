// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class SparkMaxFactory {
    
    public static class Configuration {
        public MotorType MOTOR_TYPE = MotorType.kBrushless;
        public IdleMode IDLE_MODE = IdleMode.kBrake;
        public double RAMP_RATE = 0.0;
        public boolean INVERTED = false;
    }

    private static final Configuration defaultConfig = new Configuration();

    public static CANSparkMax createDefaultSparkMax(int motorId) {
        return createSparkMax(motorId, defaultConfig);
    }

    public static CANSparkMax createSparkMax(int motorId, Configuration config) {
        CANSparkMax spark = new CANSparkMax(motorId, config.MOTOR_TYPE);
        spark.restoreFactoryDefaults();
        spark.setIdleMode(config.IDLE_MODE);
        spark.setOpenLoopRampRate(config.RAMP_RATE);
        spark.setInverted(config.INVERTED);

        return spark;
    }

    public static CANSparkMax createDefaultFollowSparkMax(int motorId, CANSparkMax original) {
        return createFollowSparkMax(motorId, original, defaultConfig);
    }

    public static CANSparkMax createFollowSparkMax(int motorId, CANSparkMax original, Configuration config) {
        CANSparkMax spark = new CANSparkMax(motorId, config.MOTOR_TYPE);
        spark.restoreFactoryDefaults();
        spark.setIdleMode(config.IDLE_MODE);
        spark.setOpenLoopRampRate(config.RAMP_RATE);
        spark.follow(original, config.INVERTED);

        return spark;
    }
}
