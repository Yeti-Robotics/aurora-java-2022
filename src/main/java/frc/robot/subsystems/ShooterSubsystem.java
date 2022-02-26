package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX shooterLeftFalcon;
    private TalonFX shooterRightFalcon;
	private SimpleMotorFeedforward shooterFeedForward;

    public enum ShooterStatus {
        FORWARD, BACKWARDS, OFF
    }

    public static ShooterStatus shooterStatus;

    public static double setPoint = 4000.0;
    public static boolean atSetPoint = false;

    // for BangBangController
    public static boolean isShooting = false;

    public ShooterSubsystem() {
        shooterLeftFalcon = new TalonFX(ShooterConstants.SHOOTER_LEFT_FALCON);
        shooterRightFalcon = new TalonFX(ShooterConstants.SHOOTER_RIGHT_FALCON);

        shooterLeftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        shooterRightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        
        shooterLeftFalcon.follow(shooterRightFalcon);
        shooterLeftFalcon.setInverted(InvertType.OpposeMaster);
        
        shooterStatus = ShooterStatus.OFF;
        
        shooterLeftFalcon.setNeutralMode(NeutralMode.Coast);
        shooterRightFalcon.setNeutralMode(NeutralMode.Coast);
        
		shooterFeedForward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA);
       
        // shooterRightFalcon.config_kP(0, ShooterConstants.SHOOTER_P);
        // shooterRightFalcon.config_kI(0, ShooterConstants.SHOOTER_I);
        // shooterRightFalcon.config_kD(0, ShooterConstants.SHOOTER_D);
        // shooterRightFalcon.config_kF(0, getFeedForward());
    
        // shooterRightFalcon.configPeakOutputForward(0.8);
    }

    @Override
    public void periodic() {
        // ShooterSubsystem.atSetPoint = Math.abs(getFlywheelRPM() - ShooterSubsystem.setPoint) <= ShooterConstants.RPM_TOLERANCE;
        ShooterSubsystem.atSetPoint = getFlywheelRPM() >= 3000.0; // FOR TESTING PURPOSES ONLY!!!!!! do above commented code ^
        SmartDashboard.putNumber("Flywheel Set Point: ", ShooterSubsystem.setPoint);
        SmartDashboard.putNumber("Flywheel Voltage", shooterRightFalcon.getMotorOutputVoltage());
        System.out.println("Flywheel RPM: " + getFlywheelRPM());
    }

    // toggles bang-bang control
    public void toggleFlywheelBB(){
        isShooting = !isShooting;
    }

    public void shootFlywheel() {
        shooterRightFalcon.set(ControlMode.PercentOutput, ShooterConstants.SHOOTER_SPEED);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void shootFlywheel(double speed) {
        shooterRightFalcon.set(ControlMode.PercentOutput, speed);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void setFlywheelVelocity(double vel){
        shooterRightFalcon.set(ControlMode.Velocity, vel);
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void reverseFlywheel() {
        shooterRightFalcon.set(ControlMode.PercentOutput, -ShooterConstants.SHOOTER_SPEED);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void reverseFlywheel(double speed) {
        shooterRightFalcon.set(ControlMode.PercentOutput, speed);
        shooterStatus = ShooterStatus.BACKWARDS;
    }

    public void stopFlywheel() {
        shooterRightFalcon.set(ControlMode.PercentOutput, 0.0);
        shooterStatus = ShooterStatus.OFF;
        ShooterSubsystem.isShooting = false;
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
        return getAverageEncoder() * ShooterConstants.PULLEY_RATIO * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION);
    }

    public static ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public double getVelocityUnitsFromRPM(double RPM) {
        return RPM / (ShooterConstants.PULLEY_RATIO * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION));
    }

    public void setSetPoint(double setPoint) {
        ShooterSubsystem.setPoint = setPoint;
        // shooterRightFalcon.config_kF(0, getFeedForward());
    }

    public double getFeedForward(){
        double kF = shooterFeedForward.calculate((ShooterSubsystem.setPoint / 60.0) * Units.inchesToMeters(ShooterConstants.FLYWHEEL_DIAMETER) * Math.PI);
        System.out.println("kF: " + kF);
        return 0; // kF;
    }
}
