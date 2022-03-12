package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX shooterLeftFalcon;
    private TalonFX shooterRightFalcon;

    public enum ShooterStatus {
        FORWARD, BACKWARDS, OFF
    }
    public static ShooterStatus shooterStatus;

    public static double setPoint = 3999.0;
    public static boolean atSetPoint = false;
    public static boolean isShooting = false;

    private PIDController highPIDController; // high RPM shooting
    private PIDController lowPIDController; // low RPM shooting

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

        highPIDController = new PIDController(ShooterConstants.HIGH_P, ShooterConstants.HIGH_I, ShooterConstants.HIGH_D);
        lowPIDController = new PIDController(ShooterConstants.LOW_P, ShooterConstants.LOW_I, ShooterConstants.LOW_D);
    }

    @Override
    public void periodic() {
        // ShooterSubsystem.atSetPoint = Math.abs(getFlywheelRPM() - ShooterSubsystem.setPoint) <= ShooterConstants.RPM_TOLERANCE;
        ShooterSubsystem.atSetPoint = getFlywheelRPM() > 1000.0; // for testing !!!!!!!!
        SmartDashboard.putNumber("Flywheel Set Point: ", ShooterSubsystem.setPoint);
        SmartDashboard.putNumber("Flywheel Voltage", shooterRightFalcon.getMotorOutputVoltage());

        if(ShooterSubsystem.isShooting){
            double setPoint = ShooterSubsystem.setPoint;
            double RPM = getFlywheelRPM();
            // 4000 RPM = set point at which we switch PIDs (value found through testing)
            shootFlywheel((setPoint < 4000.0) ? ShooterConstants.LOW_F + lowPIDController.calculate(RPM, setPoint) : ShooterConstants.HIGH_F + highPIDController.calculate(RPM, setPoint));
        } else {
            stopFlywheel();
        }
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
    }

    // adjusts RPM setpoint based on Limelight distance
    public void updateSetPoint(){
        double dist = Limelight.getDistance(); // inches
        
        double lowA = 0.416609;
        double lowH = -110.128;
        double lowB = 1.03552;
        double lowK = 2233.7;

        if(dist <= 133.0){
            ShooterSubsystem.setPoint = lowA * Math.pow(lowB, dist - lowH) + lowK;
        } else {
            // TODO
            ShooterSubsystem.setPoint = 0.0;
        }
    }
}
