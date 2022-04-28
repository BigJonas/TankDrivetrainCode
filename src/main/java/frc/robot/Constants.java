// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.util.SparkMaxFactory.Configuration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        // Order Left 1, Left 2, Right 1, Right 2
        public static final int[] MOTOR_IDS = {16, 17, 14, 10};
        public static final Configuration[] MOTOR_CONFIGS = new Configuration[4];

        static {
            MOTOR_CONFIGS[0].INVERTED = false;
            MOTOR_CONFIGS[1].INVERTED = false;
            MOTOR_CONFIGS[2].INVERTED = true; // Right side inverted cause
            MOTOR_CONFIGS[3].INVERTED = true;

        }

        public static final double TICKS_PER_REVOLUTION_LOW = 18.6; // This is old code so i dont know what these random numbers are
        public static final double TICKS_PER_REVOLUTION_HIGH = 6.4;
        public static final double RATIO_GEAR_LOW = 18.86;
        public static final double RATIO_GEAR_HIGH = 6.45;
        public static final double WHEEL_CIRCUMFERENCE_METERS = 6 * Math.PI * 0.0254; 

        public static final double KP = 0.002; // TODO: Not actual number
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double LIMITED_RATE_OF_CHANGE = 0.5; // TODO: Not actual number 

        public static final PneumaticsModuleType SHIFTER_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SHIFTER_FORWARD_CHANNEL = 4;
        public static final int SHIFTER_BACKWARD_CHANNEL = 5;

        public static final Value SHIFTER_UP_VALUE = Value.kReverse;
        public static final Value SHIFTER_DOWN_VALUE = Value.kForward;

        public static final double ROTATION_TOLERANCE = 0.5; // TODO: Not actual number

    }

    public static final class Control {
        public static final class Driver {
            public static final int PORT = 0;
            public static final double LEFT_X_DEADBAND = 0.1;
            public static final double LEFT_Y_DEADBAND = 0.1;
            public static final double RIGHT_X_DEADBAND = 0.1;
            public static final double LEFT_TRIGGER_DEADBAND = 0.5;
            public static final double RIGHT_TRIGGER_DEADBAND = 0.5;
        }
    }
}
