package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ShiftingGearsSubsystem.ShiftStatus;

public class DrivetrainSubsystem extends SubsystemBase {
  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;  
  private PigeonIMU gyro; 
  
  private DifferentialDrive drive;
  private DriveMode driveMode;

  public enum DriveMode {
    TANK, CHEEZY, ARCADE;
  }

  public DrivetrainSubsystem() {   
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);
    gyro = new PigeonIMU(DriveConstants.GYRO_ID);

    leftFalcon2.follow(leftFalcon1);
    leftFalcon2.setInverted(InvertType.FollowMaster);

    rightFalcon2.follow(rightFalcon1);
    rightFalcon1.setInverted(true);
    rightFalcon2.setInverted(InvertType.FollowMaster);

    drive = new DifferentialDrive(leftFalcon1, rightFalcon1);
    drive.setDeadband(0.05);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    leftFalcon1.setNeutralMode(NeutralMode.Coast);
    rightFalcon1.setNeutralMode(NeutralMode.Coast);

    resetEncoders();
  
    driveMode = DriveMode.CHEEZY;
  }

  @Override
  public void periodic(){}

  public void tankDrive(double leftpower, double rightpower) {
    drive.tankDrive(leftpower, rightpower);
  }

  public void cheezyDrive(double straight, double turn) {
    drive.curvatureDrive(straight, -turn, false);
  }

  public void arcadeDrive(double straight, double turn) {
    drive.arcadeDrive(straight, -turn);
  }

  public void stopDrive() {
    leftFalcon1.set(ControlMode.PercentOutput, 0.0);
    rightFalcon1.set(ControlMode.PercentOutput, 0.0);
  }
    
  public void resetEncoders() {
    leftFalcon1.setSelectedSensorPosition(0.0);
    rightFalcon1.setSelectedSensorPosition(0.0);
  }

  public double getLeftEncoder() {
    return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE)  / (ShiftingGearsSubsystem.getShifterPosition() == ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoder() {
    return (-rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (ShiftingGearsSubsystem.getShifterPosition() == ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageEncoder(){
    return ((getLeftEncoder() + getRightEncoder()) / 2.0);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getAngle(){
    double [] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void resetGyro(){
    gyro.setYaw(0.0);
  }

  public DriveMode getDriveMode(){
    return driveMode;
  }
}