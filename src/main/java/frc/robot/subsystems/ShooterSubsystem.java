package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX frontFalcon;
    private final WPI_TalonFX backFalcon;

    public enum ShooterStatus {
        FORWARD,
        BACKWARDS,
        OFF
    }

    public static ShooterStatus shooterStatus;

    public enum ShooterMode {
        HIGH_GOAL,
        LOW_GOAL
    }

    public static ShooterMode shooterMode;

    public static double frontSetPoint = 0.0;
    public static double backSetPoint = 0.0;
    public static boolean frontAtSetPoint = false;
    public static boolean backAtSetPoint = false;
    public static boolean isShooting = false;

    private PIDController frontShooterPID;
    private PIDController backShooterPID;

    public ShooterSubsystem() {
        frontFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_FRONT_FALCON);
        backFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_BACK_FALCON);

        frontFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        backFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        frontFalcon.setInverted(false);
        backFalcon.setInverted(true);

        shooterStatus = ShooterStatus.OFF;
        shooterMode = ShooterMode.HIGH_GOAL;

        frontFalcon.setNeutralMode(NeutralMode.Coast);
        backFalcon.setNeutralMode(NeutralMode.Coast);

        frontFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        backFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

        frontFalcon.enableVoltageCompensation(true);
        backFalcon.enableVoltageCompensation(true);

        frontShooterPID =
                new PIDController(
                        ShooterConstants.FRONT_SHOOTER_P, ShooterConstants.FRONT_SHOOTER_I, ShooterConstants.FRONT_SHOOTER_D);

        backShooterPID =
                new PIDController(
                        ShooterConstants.BACK_SHOOTER_P, ShooterConstants.BACK_SHOOTER_I, ShooterConstants.BACK_SHOOTER_D
                );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Set Point: ", frontSetPoint);
        SmartDashboard.putNumber("Back Set Point: ", backSetPoint);

        if (shooterStatus == ShooterStatus.OFF) {
            stopFlywheels();
            return;
        }

        if (shooterMode == ShooterMode.HIGH_GOAL) {
            // setSetPoint(some equation lol); TODO
            shootFlywheels(
                ShooterConstants.FRONT_SHOOTER_F + frontShooterPID.calculate(getFrontFlywheelSurfaceVelocity(), frontSetPoint), 
                ShooterConstants.BACK_SHOOTER_F + backShooterPID.calculate(getBackFlywheelSurfaceVelocity(), backSetPoint));
        } else {
            shootFlywheels(ShooterConstants.FRONT_LOW_SPEED, ShooterConstants.BACK_LOW_SPEED);
        }
    }

    /**
     * Runs both sides of the flywheel at a PERCENT power. 
     * @param frontSpeed the desired percent speed of the front flywheel; [-1, 1]
     * @param backSpeed the desired percent speed of the back flywheel; [-1, 1]
     */
    public void shootFlywheels(double frontSpeed, double backSpeed) {
        frontFalcon.set(ControlMode.PercentOutput, frontSpeed);
        backFalcon.set(ControlMode.PercentOutput, backSpeed);

        shooterStatus = (frontSpeed > 0.0 && backSpeed > 0.0) ? ShooterStatus.FORWARD : ShooterStatus.BACKWARDS;
    }

    /**
     * Runs both sides of the flywheel in units of ENCODER_CLICKS / 100 milliseconds. 
     * @param frontVel the desired velocity of the front flywheel, clicks per 100 ms
     * @param backVel the desired velocity of the back flywheel, clicks per 100 ms
     */
    public void setFlywheelVelocities(double frontVel, double backVel) {
        frontFalcon.set(ControlMode.Velocity, frontVel);
        backFalcon.set(ControlMode.Velocity, backVel);

        shooterStatus = (frontVel > 0.0 && backVel > 0.0) ? ShooterStatus.FORWARD : ShooterStatus.BACKWARDS;
    }

    /**
     * Stops running both flywheels. 
     */
    public void stopFlywheels() {
        frontFalcon.set(ControlMode.PercentOutput, 0.0);
        backFalcon.set(ControlMode.PercentOutput, 0.0);
        
        shooterStatus = ShooterStatus.OFF;
    }

    public double getFrontEncoder() {
        return frontFalcon.getSelectedSensorVelocity();
    }

    public double getBackEncoder() {
        return backFalcon.getSelectedSensorVelocity();
    }

    public double getFrontFlywheelRPM() {
        return getFrontEncoder()
                * ShooterConstants.FRONT_GEAR_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION);
    }

    public double getBackFlywheelRPM() {
        return getBackEncoder()
                * ShooterConstants.BACK_GEAR_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION);
    }

    /**
     * Calculates the velocity of the FRONT flywheel in meters per second. 
     * @return the velocity of the front flywheel, m/s
     */
    public double getFrontFlywheelSurfaceVelocity() {
        double rotationsPerSecond = getFrontFlywheelRPM() / 60.0;
        return rotationsPerSecond * (Math.PI * ShooterConstants.FRONT_DIAMETER_M);
    }

    /**
     * Calculates the velocity of the BACK flywheel in meters per second. 
     * @return the velocity of the back flywheel, m/s
     */
    public double getBackFlywheelSurfaceVelocity() {
        double rotationsPerSecond = getBackFlywheelRPM() / 60.0;
        return rotationsPerSecond * (Math.PI * ShooterConstants.BACK_DIAMETER_M);
    }

    public double getMetersPerSecondFromRPM(double RPM) {
        return (ShooterConstants.FRONT_DIAMETER_M * Math.PI) * (RPM / 60.0);
    }

    public static ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public double getVelocityUnitsFromRPM(double RPM) {
        return RPM
                / (ShooterConstants.BACK_GEAR_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION));
    }

    /**
     * Tests if both flywheels have reached their setpoints. 
     * @return true if both flywheels are up to speed; false otherwise
     */
    public static boolean atSetPoint() {
        return shooterStatus == ShooterStatus.FORWARD;

        // we should do this properly if our PID is minty enough
        // return frontAtSetPoint && backAtSetPoint;
    }

    /**
     * Sets the set point of both flywheels with DISTINCT values, in units of meters per second. 
     * @param frontVel the desired velocity of the front flywheel, m/s
     * @param backVel the desired velocity of the back flywheel, m/s
     */
    public void setSetPoints(double frontVel, double backVel) {
        frontSetPoint = frontVel;
        backSetPoint = backVel; 
    }

    /**
     * Sets the set point of both flywheels with the SAME values, in units of meters per second. 
     * @param vel the desired velocity of both flywheels, m/s
     */
    public void setSetPoint(double vel) {
        frontSetPoint = vel;
        backSetPoint = vel;
    }
}
