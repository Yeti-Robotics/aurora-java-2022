// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {

        public static final int LEFT_FALCON_1 = 1;//placeholder
        public static final int LEFT_FALCON_2 = 2;//placeholder
        public static final int LEFT_FALCON_3 = 3;//placeholder

        public static final int RIGHT_FALCON_1 = 4;//placeholder
        public static final int RIGHT_FALCON_2 = 5;//placeholder
        public static final int RIGHT_FALCON_3 = 6;//placeholder

        public static final int[] SOLENOID_SHIFTER = {0, 1};//placeholder

        public static final double DRIVE_ENCODER_RESOLUTION = 2048;//placeholder
        public static final double DRIVE_WHEEL_DIAMETER = 4;//placeholder
        public static final double DISTANCE_PER_PULSE = (DRIVE_WHEEL_DIAMETER * Math.PI) / DRIVE_ENCODER_RESOLUTION;

        public static final double HIGH_GEAR_RATIO = 1.0;//placeholder
        public static final double LOW_GEAR_RATIO = 2.0;//placeholder
    }
    
    public static final class ClimberConstants {

        public static final int RIGHT_CLIMBER = 0; //placeholder
        public static final int LEFT_CLIMBER = 0; //placeholder

        public static final double CLIMB_SPEED = 0.8; //placeholder

        public static final int[] CLIMBER_BRAKE_SOLENOID = {0, 1}; //placeholder

    }
}
