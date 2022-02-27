// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX climberFalcon1, climberFalcon2;
  private TalonSRX climberWinch;
  private DoubleSolenoid climberStationaryHooks;
  private DoubleSolenoid climberMovingHook;
  private DoubleSolenoid climberLeanPiston;

  public ClimberSubsystem() {
    climberFalcon1 = new WPI_TalonFX(ClimberConstants.CLIMBER_1);
    climberFalcon2 = new WPI_TalonFX(ClimberConstants.CLIMBER_2);
    climberWinch = new TalonSRX(ClimberConstants.CLIMBER_WINCH);
    
    climberStationaryHooks = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_STATIONARY_PISTONS[0], ClimberConstants.CLIMBER_STATIONARY_PISTONS[1]);
    climberMovingHook = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_MOVING_PISTON[0], ClimberConstants.CLIMBER_MOVING_PISTON[1]);
    climberLeanPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_LEAN_PISTON[0], ClimberConstants.CLIMBER_LEAN_PISTON[1]);

    climberStationaryHooks.set(Value.kReverse);
    climberMovingHook.set(Value.kReverse);
    climberLeanPiston.set(Value.kReverse);

    climberFalcon1.setInverted(true);
    climberFalcon2.follow(climberFalcon1);
    climberFalcon2.setInverted(InvertType.FollowMaster);

    climberFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberFalcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    climberFalcon1.setNeutralMode(NeutralMode.Brake);
    climberFalcon2.setNeutralMode(NeutralMode.Brake);
    climberWinch.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {}

  public void climbUp() {
    climberFalcon1.set(ControlMode.PercentOutput, ClimberConstants.CLIMB_SPEED);
  }
  public void climbDown(){
    climberFalcon1.set(ControlMode.PercentOutput, -ClimberConstants.CLIMB_SPEED);
  }

  public void stopClimb(){
    climberFalcon1.set(ControlMode.PercentOutput, 0.0);
  }

  public void toggleStaticHooks(){
    climberStationaryHooks.toggle();
  }

  public void toggleMovingHook(){
    climberMovingHook.toggle();
  }

  public void toggleLeanPiston(){
    climberLeanPiston.toggle();
  }

  public double getLeftEncoder(){
    return climberFalcon2.getSelectedSensorPosition();
  }

  public double getRightEncoder(){
    return climberFalcon1.getSelectedSensorPosition();
  }

  public double getAverageEncoder(){
    return (getLeftEncoder() + getRightEncoder()) / 2.0;
  }

  public void resetEncoders(){
    climberFalcon2.setSelectedSensorPosition(0.0);
    climberFalcon1.setSelectedSensorPosition(0.0);
  }

  public void moveWinch(double power){
    climberWinch.set(ControlMode.PercentOutput, power);
  }

  public void winchIn(){
    climberWinch.set(ControlMode.PercentOutput, ClimberConstants.WINCH_SPEED);
  }

  public void winchOut(){
    climberWinch.set(ControlMode.PercentOutput, -ClimberConstants.WINCH_SPEED);
  }

  public void stopWinch(){
    climberWinch.set(ControlMode.PercentOutput, 0.0);
  }
}
