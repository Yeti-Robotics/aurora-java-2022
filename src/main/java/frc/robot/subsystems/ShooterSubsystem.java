package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

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

    public static double setPoint = 0.0;
    public static boolean atSetPoint = false;
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
        ShooterSubsystem.atSetPoint = shooterStatus == ShooterStatus.FORWARD;
        SmartDashboard.putNumber("Flywheel Set Point: ", ShooterSubsystem.setPoint);
        SmartDashboard.putNumber("Flywheel Voltage", backFalcon.getMotorOutputVoltage());

        if (shooterStatus == ShooterStatus.OFF) {
            stopFlywheel();
            return;
        }

        if (shooterMode == ShooterMode.HIGH_GOAL) {
            if (Limelight.getDistance() > 0.0) {
                ShooterSubsystem.setPoint = ((25 / 3) * Limelight.getDistance()) + 2991.66667;
            }
            shootFlywheel(
                    ShooterConstants.SHOOTER_F
                            + shooterPID.calculate(getFlywheelRPM(), ShooterSubsystem.setPoint));
        } else {
            shootFlywheel(ShooterConstants.SHOOTER_LOW_SPEED);
        }
    }

    public void shootFlywheel(double speed) {
        backFalcon.set(ControlMode.PercentOutput, speed);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void setFlywheelVelocity(double vel) {
        backFalcon.set(ControlMode.Velocity, vel);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void setFlywheelVoltage(double volts) {
        shooterFalcons.setVoltage(volts);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void reverseFlywheel(double speed) {
        backFalcon.set(ControlMode.PercentOutput, speed);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopFlywheel() {
        backFalcon.set(ControlMode.PercentOutput, 0.0);
        // shooterFalcons.set(0.0);
        shooterStatus = ShooterStatus.OFF;
    }

    public double getLeftEncoder() {
        return frontFalcon.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return backFalcon.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public double getFlywheelRPM() {
        return getAverageEncoder()
                * ShooterConstants.PULLEY_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION);
    }

    public double getFrontFlywheelSurfaceVelocity() {
        return frontFalcon.getSelectedSensorVelocity() * 10 *
    }

    public double getMetersPerSecondFromRPM(double RPM) {
        return (ShooterConstants.FLYWHEEL_DIAMETER_M * Math.PI) * (RPM / 60.0);
    }

    public static ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public double getVelocityUnitsFromRPM(double RPM) {
        return RPM
                / (ShooterConstants.PULLEY_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION));
    }

    public void setSetPoint(double setPoint) {
        ShooterSubsystem.setPoint = setPoint;
    }

    // returns in volts
    public double getFeedForward() {
        return (Constants.MOTOR_VOLTAGE_COMP / 8750.0) * ShooterSubsystem.setPoint;
    }
}
