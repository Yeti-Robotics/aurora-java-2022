package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX frontFalcon;
  private TalonFX backFalcon;

  private MotorControllerGroup shooterFalcons;

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

  private PIDController shooterPID;

  public ShooterSubsystem() {
    frontFalcon = new TalonFX(ShooterConstants.SHOOTER_FRONT_FALCON);
    backFalcon = new TalonFX(ShooterConstants.SHOOTER_BACK_FALCON);

    // shooterFalcons = new MotorControllerGroup(shooterLeftFalcon, shooterRightFalcon);

    frontFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    backFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    frontFalcon.follow(backFalcon);
    frontFalcon.setInverted(InvertType.OpposeMaster);

    shooterStatus = ShooterStatus.OFF;
    shooterMode = ShooterMode.HIGH_GOAL;

    frontFalcon.setNeutralMode(NeutralMode.Coast);
    backFalcon.setNeutralMode(NeutralMode.Coast);

    frontFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    backFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

    frontFalcon.enableVoltageCompensation(true);
    backFalcon.enableVoltageCompensation(true);

    shooterPID =
        new PIDController(
            ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);
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

    if (shooterMode == ShooterMode.HIGH_GOAL){
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
