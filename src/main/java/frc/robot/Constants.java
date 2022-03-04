// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

        public static final int[] SOLENOID_SHIFTER = {0, 1};

        public static final double DRIVE_ENCODER_RESOLUTION = 2048.0;
        public static final double DRIVE_WHEEL_DIAMETER = 4.0;
        public static final double DISTANCE_PER_PULSE = (DRIVE_WHEEL_DIAMETER * Math.PI) / DRIVE_ENCODER_RESOLUTION;

        public static final double HIGH_GEAR_RATIO = 1.0;//placeholder
        public static final double LOW_GEAR_RATIO = 2.0;//placeholder
    }

    public static final class LEDConstants {
        public static final int ADDRESSABLE_LED = 1;
        public static final int LED_COUNT = 120;
    }
    
    public static final class ClimberConstants {
        public static final int CLIMBER_1 = 5; // front relative to robot
        public static final int CLIMBER_2 = 6; // rear relative to robot

        public static final int CLIMBER_WINCH = 3; //placeholder

        public static final double CLIMB_SPEED = 0.3;
        public static final double CLIMBER_LOWER_LIMIT = 0.0;
        public static final double CLIMBER_UPPER_LIMIT = 101331.5; 
        public static final double CLIMBER_TOLERANCE = 15.0; 
        public static final double CLIMBER_WINCH_SPEED = 0.8    ;

        public static final int[] CLIMBER_LEAN_PISTON = {4, 5}; 
        public static final int[] CLIMBER_MOVING_PISTON = {6, 7}; 
        public static final int[] CLIMBER_STATIONARY_PISTONS = {8, 9}; 

    }

    public static final class ShooterConstants {
        //shooter motor ports
        public static final int SHOOTER_LEFT_FALCON = 8; //left
        public static final int SHOOTER_RIGHT_FALCON = 7; //right

        // high RPM PID constants
        public static final double HIGH_P = 0.0012;
        public static final double HIGH_I = 0.000056;
        public static final double HIGH_D = 0.0000495;
        public static final double HIGH_F = 0.45; 

        // low RPM PID constants
        public static final double LOW_P = 0.0004;
        public static final double LOW_I = 0.0; 
        public static final double LOW_D = 0.00002;
        public static final double LOW_F = 0.37;

        public static final double SHOOTER_MAX_VEL = 12242.5; // in native encoder units per 100 ms

        //feed forward values
        public static final double SHOOTER_KS = 0.72828;
        public static final double SHOOTER_KV = 0.15313;
        public static final double SHOOTER_KA = 0.033071;

        //shooter motor speeds    
        public static final double SHOOTER_SPEED = 0.6;

        //shooter rpm calc constants
        public static final double PULLEY_RATIO = 48.0 / 36.0; //not completely known
        public static final double ENCODER_TIME_CONVERSION = 600.0; // 100 ms per minute
        public static final double ENCODER_RESOLUTION = 2048.0;
        public static final double QUAD_FACTOR = 4.0; // quadrature encoder factor
        public static final double RPM_TOLERANCE = 10.0;

        public static final double FLYWHEEL_DIAMETER = 4.0; // inches
    }

    public static final class CalcConstants {
        // distance calc constants
        public static final double KNOWN_DISTANCE = 161.3; //inches
        public static final int PIXEL_WIDTH_KNOWN = 65; //pixels
        public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; //inches
        public static final double FOCAL_LENGTH = ( KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;

        //trajectory constants
        public static final double LIMELIGHT_HEIGHT = 37.5; // inches
        public static final double GOAL_HEIGHT = 108.0; // inches
        public static final double GRAVITY = 386.09; // inches/ sec ^2
        public static final double MOUNTING_ANGLE = 33.47; // deg
    }

    public static final class IntakeConstants {
        public static final int INTAKE_FALCON = 9;
        public static final int[] INTAKE_PISTONS_SOLENOID = {2, 3}; 
        public static final double INTAKE_SPEED = 0.225; 
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
        public static final double TURRET_SPEED = 0.15;
        public static final double TURRET_P = 0.0224; 
        public static final double TURRET_I = 0.0; 
        public static final double TURRET_D = 0.0; 
        public static final double TURRET_F = 0.0;
        public static final double TURRET_MAX_RIGHT = 54.59563446044922; 
        public static final double TURRET_MAX_LEFT = -54.59563446044922;
        public static final double TURRET_TOLERANCE = 1.0; // tolerance for checking encoder limits
        public static final double LIMELIGHT_TOLERANCE = 0.5; // tolerance for alignment of target using limelight
    }

    public static final class AutoConstants {
        public static final double AUTO_KS = 0.72606; // volts
        public static final double AUTO_KV = 2.136; // volt seconds per meter
        public static final double AUTO_KA = 0.57982; // volt seconds squared per meter
        public static final double AUTO_P = 0.0;

        public static final double RAMSETE_B = 2.0;
        public static final double RAMSETE_ZETA = 0.7;

        public static final double TRACK_WIDTH = 0.0; // m
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
        
        public static final double MAX_SPEED = 3.0; // m/s
        public static final double MAX_ACCELERATION = 3.0; // m/s^2
    }

    public static final class OIConstants {
        public static final int DRIVER_STATION_JOY = 0;
    }
}
