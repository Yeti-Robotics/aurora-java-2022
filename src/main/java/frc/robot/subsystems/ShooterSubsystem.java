package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX shooterLeftFalcon;
  private final WPI_TalonFX shooterRightFalcon;

  private final MotorControllerGroup shooterFalcons;

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
  private final SimpleMotorFeedforward feedForward;

  public ShooterSubsystem(
          WPI_TalonFX shooterLeft,
          WPI_TalonFX shooterRight,
          PIDController shooterPID,
          SimpleMotorFeedforward feedForward
  ) {
    shooterLeftFalcon = shooterLeft;
    shooterRightFalcon = shooterRight;

    shooterFalcons = new MotorControllerGroup(shooterLeftFalcon, shooterRightFalcon);

    shooterLeftFalcon.follow(shooterRightFalcon);
    shooterLeftFalcon.setInverted(InvertType.OpposeMaster);

    shooterStatus = ShooterStatus.OFF;
    shooterMode = ShooterMode.LIMELIGHT;

    this.shooterPID = shooterPID;
    this.feedForward = feedForward;
  }

  @Override
  public void periodic() {
    ShooterSubsystem.atSetPoint = shooterStatus == ShooterStatus.FORWARD;
    SmartDashboard.putNumber("Flywheel Set Point: ", ShooterSubsystem.setPoint);
    SmartDashboard.putNumber("Flywheel Voltage", shooterRightFalcon.getMotorOutputVoltage());

    System.out.println("RPM: " + getFlywheelRPM());

    if (ShooterSubsystem.isShooting) {
      switch (shooterMode) {
        case LIMELIGHT:
          // if (PhotonVision.getDistance() > 0.0) {
          // ShooterSubsystem.setPoint = ((25 / 3) * PhotonVision.getDistance()) +
          // 2991.66667;
          // }
          if (Limelight.getDistance() > 0.0) {
            ShooterSubsystem.setPoint = ((25 / 3.0) * Limelight.getDistance()) + 2991.66667;
          }
          shootFlywheel(
              ShooterConstants.SHOOTER_F
                  + shooterPID.calculate(getFlywheelRPM(), ShooterSubsystem.setPoint));

          break;
        case LAUNCHPAD:
          shootFlywheel(
              ShooterConstants.SHOOTER_F
                  + shooterPID.calculate(getFlywheelRPM(), ShooterSubsystem.setPoint));
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
