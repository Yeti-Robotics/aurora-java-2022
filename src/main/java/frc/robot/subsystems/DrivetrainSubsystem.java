package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ShiftingSubsystem.ShiftStatus;
import javax.inject.Inject;
import javax.inject.Named;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftFalcon1;
  private final WPI_TalonFX leftFalcon2;
  private final WPI_TalonFX rightFalcon1;
  private final WPI_TalonFX rightFalcon2;
  private final MotorControllerGroup leftMotors;
  private final MotorControllerGroup rightMotors;

  private final AHRS gyro;

  private final DifferentialDrive drive;
  private DriveMode driveMode;
  private final DifferentialDriveOdometry odometry;

  private Command tankCommand;
  private Command cheezyCommand;
  private Command arcadeCommand;

  private final PIDController drivePID;

  public enum DriveMode {
    TANK,
    CHEEZY,
    ARCADE
  }

  private NeutralMode neutralMode;

  @Inject
  public DrivetrainSubsystem(
      @Named("tank drive") Command tankCommand,
      @Named("cheezy drive") Command cheezyCommand,
      @Named("arcade drive") Command arcadeCommand) {
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    leftFalcon1.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    leftFalcon2.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    rightFalcon1.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    rightFalcon2.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

    leftFalcon1.enableVoltageCompensation(true);
    leftFalcon2.enableVoltageCompensation(true);
    rightFalcon1.enableVoltageCompensation(true);
    rightFalcon2.enableVoltageCompensation(true);

    leftMotors = new MotorControllerGroup(leftFalcon1, leftFalcon2);
    rightMotors = new MotorControllerGroup(rightFalcon1, rightFalcon2);
    rightMotors.setInverted(true);
    leftMotors.setInverted(false);
    setMotorsBrake();

    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(0.0525);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    resetEncoders();

    gyro = new AHRS(Port.kUSB);
    resetGyro();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    driveMode = DriveMode.CHEEZY;
    this.tankCommand = tankCommand;
    this.cheezyCommand = cheezyCommand;
    this.arcadeCommand = arcadeCommand;
    setDriveMode(DriveMode.CHEEZY);

    drivePID =
        new PIDController(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void tankDrive(double leftpower, double rightpower) {
    drive.tankDrive(leftpower, rightpower);
  }

  public void cheezyDrive(double straight, double turn) {
    drive.curvatureDrive(straight, -turn, true);
  }

  public void arcadeDrive(double straight, double turn) {
    drive.arcadeDrive(straight, -turn);
  }

  public void stopDrive() {
    leftMotors.set(0.0);
    rightMotors.set(0.0);
  }

  public void setMotorsBrake() {
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    leftFalcon2.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon2.setNeutralMode(NeutralMode.Brake);
    neutralMode = NeutralMode.Brake;
  }

  public void setMotorsCoast() {
    leftFalcon1.setNeutralMode(NeutralMode.Coast);
    leftFalcon2.setNeutralMode(NeutralMode.Coast);
    rightFalcon1.setNeutralMode(NeutralMode.Coast);
    rightFalcon2.setNeutralMode(NeutralMode.Coast);
    neutralMode = NeutralMode.Coast;
  }

  // Distance in meters
  public double getLeftEncoderDistance() {
    return (leftFalcon1.getSelectedSensorPosition()
        * (DriveConstants.DISTANCE_PER_PULSE)
        / (ShiftingSubsystem.getShifterPosition() == ShiftStatus.HIGH
            ? DriveConstants.HIGH_GEAR_RATIO
            : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoderDistance() {
    return (-rightFalcon1.getSelectedSensorPosition()
        * (DriveConstants.DISTANCE_PER_PULSE)
        / (ShiftingSubsystem.getShifterPosition() == ShiftStatus.HIGH
            ? DriveConstants.HIGH_GEAR_RATIO
            : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageEncoder() {
    return ((getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0);
  }

  public void resetEncoders() {
    leftFalcon1.setSelectedSensorPosition(0.0);
    rightFalcon1.setSelectedSensorPosition(0.0);
  }

  // Velocity in meters/second
  public double getLeftEncoderVelocity() {
    return (leftFalcon1.getSelectedSensorVelocity()
        * 10
        * (DriveConstants.DISTANCE_PER_PULSE)
        / (ShiftingSubsystem.getShifterPosition() == ShiftStatus.HIGH
            ? DriveConstants.HIGH_GEAR_RATIO
            : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoderVelocity() {
    return (-rightFalcon1.getSelectedSensorVelocity()
        * 10
        * (DriveConstants.DISTANCE_PER_PULSE)
        / (ShiftingSubsystem.getShifterPosition() == ShiftStatus.HIGH
            ? DriveConstants.HIGH_GEAR_RATIO
            : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageVelocity() {
    return (getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2.0;
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setDriveMode(DriveMode driveMode) {
    this.driveMode = driveMode;
    switch (this.driveMode) {
      case TANK:
        this.setDefaultCommand(cheezyCommand);
        break;
      case CHEEZY:
        this.setDefaultCommand(cheezyCommand);
        break;
      case ARCADE:
        this.setDefaultCommand(arcadeCommand);
        break;
    }
  }

  public NeutralMode getNeutralMode() {
    return neutralMode;
  }

  public void turnToAnglePID(double angle) {
    double output = drivePID.calculate(getHeading(), angle);
    System.out.println("OUTPUT: " + output);
    cheezyDrive(0.0, output);
  }
}
