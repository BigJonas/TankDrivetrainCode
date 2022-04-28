// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
public class TalonFactory {

    public static class Configuration {
        public NeutralMode IDLE_MODE = NeutralMode.Brake;
        public double RAMP_RATE = 0.0;
        public boolean INVERTED = false;
    }

    private static Configuration defaultConfig = new Configuration();

    public static WPI_TalonSRX createTalonSRX(int motorId, Configuration config) {
        WPI_TalonSRX talon = new WPI_TalonSRX(motorId);
        talon.configFactoryDefault();
        talon.setNeutralMode(config.IDLE_MODE);
        talon.configOpenloopRamp(config.RAMP_RATE);
        talon.setInverted(config.INVERTED);

        return talon;
    }

    public static WPI_TalonSRX createDefaultTalonSRX(int motorId) {
        return createTalonSRX(motorId, defaultConfig);
    }

    public static WPI_TalonFX createFalcon(int motorId, Configuration config) {
        WPI_TalonFX falcon = new WPI_TalonFX(motorId);
        falcon.configFactoryDefault();
        falcon.setNeutralMode(config.IDLE_MODE);
        falcon.configOpenloopRamp(config.RAMP_RATE);
        falcon.setInverted(config.INVERTED);

        return falcon;
    }

    public static WPI_TalonFX createDefaultFalcon(int motorId) {
        return createFalcon(motorId, defaultConfig);
    }
}
