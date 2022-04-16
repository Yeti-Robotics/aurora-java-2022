package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.PhotonVision;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterLeftFalcon;
    private final TalonFX shooterRightFalcon;

    private MotorControllerGroup shooterFalcons;

    public enum ShooterStatus {
        FORWARD,
        BACKWARDS,
        OFF
    }

    public static ShooterStatus shooterStatus;

    public enum ShooterMode {
        LIMELIGHT,
        LAUNCHPAD,
        LOWGOAL
    }

    public static ShooterMode shooterMode;

    public static double setPoint = 0.0;
    public static boolean atSetPoint = false;
    public static boolean isShooting = false;

    private final PIDController shooterPID;

    public ShooterSubsystem() {
        shooterLeftFalcon = new TalonFX(ShooterConstants.SHOOTER_LEFT_FALCON);
        shooterRightFalcon = new TalonFX(ShooterConstants.SHOOTER_RIGHT_FALCON);

        // shooterFalcons = new MotorControllerGroup(shooterLeftFalcon,
        // shooterRightFalcon);

        shooterLeftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        shooterRightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        shooterLeftFalcon.follow(shooterRightFalcon);
        shooterLeftFalcon.setInverted(InvertType.OpposeMaster);

        // shooterLeftFalcon.setInverted(true);

        shooterStatus = ShooterStatus.OFF;
        shooterMode = ShooterMode.LIMELIGHT;

        shooterLeftFalcon.setNeutralMode(NeutralMode.Coast);
        shooterRightFalcon.setNeutralMode(NeutralMode.Coast);

        shooterLeftFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        shooterRightFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

        shooterLeftFalcon.enableVoltageCompensation(true);
        shooterRightFalcon.enableVoltageCompensation(true);

        shooterPID =
                new PIDController(
                        ShooterConstants.SHOOTER_P,
                        ShooterConstants.SHOOTER_I,
                        ShooterConstants.SHOOTER_D);
    }

    @Override
    public void periodic() {
        ShooterSubsystem.atSetPoint = shooterStatus == ShooterStatus.FORWARD;
        SmartDashboard.putNumber("Flywheel Set Point: ", ShooterSubsystem.setPoint);
        SmartDashboard.putNumber("Flywheel Voltage", shooterRightFalcon.getMotorOutputVoltage());

        if (ShooterSubsystem.isShooting) {
            switch (shooterMode) {
                case LIMELIGHT:
                    if (PhotonVision.getDistance() > 0.0) {
                        ShooterSubsystem.setPoint =
                                ((25 / 3) * PhotonVision.getDistance()) + 2991.66667;
                    }
                    shootFlywheel(
                            ShooterConstants.SHOOTER_F
                                    + shooterPID.calculate(
                                    getFlywheelRPM(), ShooterSubsystem.setPoint));
                    break;
                case LAUNCHPAD:
                    shootFlywheel(
                            ShooterConstants.SHOOTER_F
                                    + shooterPID.calculate(
                                    getFlywheelRPM(), ShooterSubsystem.setPoint));
                    break;
                case LOWGOAL:
                    shootFlywheel(ShooterConstants.SHOOTER_LOW_SPEED);
                    break;
            }
        } else {
            stopFlywheel();
        }
    }

    public void shootFlywheel(double speed) {
        shooterRightFalcon.set(ControlMode.PercentOutput, speed);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void setFlywheelVelocity(double vel) {
        shooterRightFalcon.set(ControlMode.Velocity, vel);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void setFlywheelVoltage(double volts) {
        shooterFalcons.setVoltage(volts);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void reverseFlywheel(double speed) {
        shooterRightFalcon.set(ControlMode.PercentOutput, speed);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopFlywheel() {
        shooterRightFalcon.set(ControlMode.PercentOutput, 0.0);
        // shooterFalcons.set(0.0);
        shooterStatus = ShooterStatus.OFF;
    }

    public double getLeftEncoder() {
        return shooterLeftFalcon.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return shooterRightFalcon.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public double getFlywheelRPM() {
        return getAverageEncoder()
                * ShooterConstants.PULLEY_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION);
    }

    public static ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public double getVelocityUnitsFromRPM(double RPM) {
        return RPM
                / (ShooterConstants.PULLEY_RATIO
                * (ShooterConstants.ENCODER_TIME_CONVERSION
                / ShooterConstants.ENCODER_RESOLUTION));
    }

    public void setSetPoint(double setPoint) {
        ShooterSubsystem.setPoint = setPoint;
    }

    // returns in volts
    public double getFeedForward() {
        return (Constants.MOTOR_VOLTAGE_COMP / 8750.0) * ShooterSubsystem.setPoint;
    }
}
