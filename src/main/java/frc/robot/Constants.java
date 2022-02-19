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

        public static final int LEFT_FALCON_1 = 1; //front relative to robot
        public static final int LEFT_FALCON_2 = 2; // rear relative to robot

        public static final int RIGHT_FALCON_1 = 3; //front relative to robot
        public static final int RIGHT_FALCON_2 = 4; // rear relative to robot

        public static final int GYRO_ID = 1;

        public static final int[] SOLENOID_SHIFTER = {0,1};

        public static final double DRIVE_ENCODER_RESOLUTION = 2048.0;
        public static final double DRIVE_WHEEL_DIAMETER = 4.0;
        public static final double DISTANCE_PER_PULSE = (DRIVE_WHEEL_DIAMETER * Math.PI) / DRIVE_ENCODER_RESOLUTION;

        public static final double HIGH_GEAR_RATIO = 1.0;//placeholder
        public static final double LOW_GEAR_RATIO = 2.0;//placeholder
    }

    public static final class LEDConstants {
        public static final int ADDRESSABLE_LED = 0;
        public static final int LED_COUNT = 180;
    }
    
    public static final class ClimberConstants {
        public static final int CLIMBER_1 = 5; //front relative to robot
        public static final int CLIMBER_2 = 6; // rear relative to robot

        public static final double CLIMB_SPEED = 0.8; //placeholder

        public static final int[] CLIMBER_HOOK_PISTONS_1 = {6, 7}; // placeholder
        public static final int[] CLIMBER_HOOK_PISTONS_2 = {8, 9}; // placeholder
        public static final int[] CLIMBER_LEAN_PISTON = {4, 5}; 

    }

    public static final class ShooterConstants {
        //shooter motor ports
        public static final int SHOOTER_LEFT_FALCON = 8; //left
        public static final int SHOOTER_RIGHT_FALCON = 7; //right

        //PID constants
        public static final double SHOOTER_P = 0.0;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_F = 0.0;

        //shooter motor speeds    
        public static final double SHOOTER_SPEED = 0.7;

        //shooter rpm calc constants
        public static final double PULLEY_RATIO = 48.0 / 36.0; //not completely known
        public static final double ENCODER_TIME_CONVERSION = 600.0; // 100 ms per minute
        public static final double ENCODER_RESOLUTION = 2048.0;
        public static final double QUAD_FACTOR = 4.0; // quadrature encoder factor
        public static final double RPM_TOLERANCE = 10.0;
    }

    public static final class CalcConstants {
        // distance calc constants
        public static final double KNOWN_DISTANCE = 161.3; //inches
        public static final int PIXEL_WIDTH_KNOWN = 65; //pixels
        public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; //inches
        public static final double FOCAL_LENGTH = ( KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;

        //trajectory constants
        public static final int SHOOTER_HEIGHT = 23; // inches
        public static final double GRAVITY = 386.09; // inches/ sec ^2
    }

    public static final class IntakeConstants {
        public static final int INTAKE_FALCON = 9;
        public static final int[] INTAKE_PISTONS_SOLENOID = {2, 3}; 
        public static final double INTAKE_SPEED = 0.2; 
    }

    public static final class NeckConstants{
        public static final int FRONT_INDEXER = 10;
        public static final int REAR_INDEXER = 11;   
        public static final int NECK_LOWER_BEAM_BREAK = 4;
        public static final int NECK_UPPER_BEAM_BREAK = 2;
        public static final double NECK_FRONT_SPEED = 0.6;
        public static final double NECK_REAR_SPEED = 0.6;
    }

    public static final class TurretConstants {
        public static final int TURRET_SPARK = 12; 
        public static final int MAG_SWITCH_PORT = 3;
        public static final double TURRET_SPEED = 0.5;
        public static final double kPTurretVel = 0.0; // ph
        public static final double kITurretVel = 0.0; // ph
        public static final double kDTurretVel = 0.0; // ph
        public static final double TURRET_MAX_RIGHT = 90; //placeholder
        public static final double TURRET_MAX_LEFT = -90; //placeholder
        public static final double TURRET_MAX_TOLERANCE = 10;
    }

    public static final class OIConstants {
        public static final int DRIVER_STATION_JOY = 0;
    }
}
