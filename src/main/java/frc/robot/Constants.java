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

        public static final int LEFT_FALCON_1 = 13;//placeholder
        public static final int LEFT_FALCON_2 = 16;//placeholder
        public static final int LEFT_FALCON_3 = 7;//placeholder

        public static final int RIGHT_FALCON_1 = 3;//placeholder
        public static final int RIGHT_FALCON_2 = 8;//placeholder
        public static final int RIGHT_FALCON_3 = 6;//placeholder

        public static final int[] SOLENOID_SHIFTER = {6,7};//placeholder

        public static final double DRIVE_ENCODER_RESOLUTION = 2048;//placeholder
        public static final double DRIVE_WHEEL_DIAMETER = 4;//placeholder
        public static final double DISTANCE_PER_PULSE = (DRIVE_WHEEL_DIAMETER * Math.PI) / DRIVE_ENCODER_RESOLUTION;

        public static final double HIGH_GEAR_RATIO = 1.0;//placeholder
        public static final double LOW_GEAR_RATIO = 2.0;//placeholder
    }

    public static final class LEDConstants {
        public static final int ADDRESSABLE_LED = 0;
        public static final int LED_COUNT = 180;
    }
    
    public static final class ClimberConstants {

        public static final int RIGHT_CLIMBER = 0; //placeholder
        public static final int LEFT_CLIMBER = 0; //placeholder

        public static final double CLIMB_SPEED = 0.8; //placeholder

        public static final int[] CLIMBER_BRAKE_SOLENOID = {0, 1}; //placeholder

    }

    public static final class ShooterConstants{
        //shooter motor ports
        public static final int SHOOTER_LEFT_TALON = 18; //left
        public static final int SHOOTER_RIGHT_TALON = 1; //right

        //shooter motor speeds    
        public static final double SHOOT_1_SPEED = .9;
        public static final double SHOOT_2_SPEED = .9;
        public static final double REVERSE_SHOOT_1_SPEED = -0.5;
        public static final double REVERSE_SHOOT_2_SPEED = -0.5;

        //shooter rpm calc constants
        public static final double PULLEY_RATIO = 48.0 / 36.0; //not completely known
        public static final double ENCODER_TIME_CONVERSION = 600.0; // 100 ms per minute
        public static final double ENCODER_RESOLUTION = 2048.0;
        public static final double QUAD_FACTOR = 4.0; // quadrature encoder factor
        public static final double MAX_RPM = 4000.0;
        public static final double RPM_TOLERANCE = 10.0;
    }
    public static final class CalcConstants{
        // distance calc constants
        public static final double KNOWN_DISTANCE = 161.3; //inches
        public static final int PIXEL_WIDTH_KNOWN = 65; //pixels
        public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; //inches
        public static final double FOCAL_LENGTH = ( KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;

        //trajectory constants
        public static final int SHOOTER_HEIGHT = 23; // inches
        public static final double GRAVITY = 386.09; // inches/ sec ^2
    }
    public static final class IntakeConstants{
        //intake motor port
        public static final int INTAKE_FALCON = 19;

        //intake piston solenoid ports
        public static final int[] INTAKE_PISTONS_SOLENOID = {0, 1}; 

        //intake motor speed
        public static final double ROLL_IN_SPEED = .5; //placeholder
        public static final double ROLL_OUT_SPEED = -.5;//placeholder
    }
    public static final class NeckConstants{
        //neck motor ports
        public static final int NECK_MOTOR_FRONT = 17; //placeholder
        public static final int NECK_MOTOR_BACK = 15;//PLACEHOLDER   `
        public static final int NECK_BEAMBREAK = 0;

        //beam break sensor dio ports
        public static final int NECK_BEAM_BREAK = 0;

        //neck motor speed
        public static final double NECK_UP_SPEED = 0.75;
        public static final double NECK_DOWN_SPEED = -0.5;
    }
    public static final class TurretConstants {
        public static final int TURRET_SPARK = 13; // placeholder
    }
}
