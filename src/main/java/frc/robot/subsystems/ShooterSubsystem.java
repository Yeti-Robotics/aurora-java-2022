package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
        private TalonFX shooterLeftFalcon;
        private TalonFX shooterRightFalcon;

        public enum ShooterStatus {
            FORWARD, BACKWARDS, OFF
        }

        public static ShooterStatus shooterStatus;
    
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
        }
    
        public void shootFlywheel() {
            shooterRightFalcon.set(ControlMode.PercentOutput, ShooterConstants.SHOOTER_SPEED);
            shooterStatus = ShooterStatus.FORWARD;
        }
    
        public void shootFlywheel(double speed){
            shooterRightFalcon.set(ControlMode.PercentOutput, speed);
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
            return (getRightEncoder() * ShooterConstants.PULLEY_RATIO * ShooterConstants.ENCODER_TIME_CONVERSION) / (ShooterConstants.ENCODER_RESOLUTION * ShooterConstants.QUAD_FACTOR);
        }
    
        public static ShooterStatus getShooterStatus(){
            return shooterStatus;
        }
    
        public double getVelocityUnitsFromRPM(double RPM){
            return RPM / (ShooterConstants.PULLEY_RATIO * (ShooterConstants.ENCODER_TIME_CONVERSION / ShooterConstants.ENCODER_RESOLUTION));
        }
    }
        

