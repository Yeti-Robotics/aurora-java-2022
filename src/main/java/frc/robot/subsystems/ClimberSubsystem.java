// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX climberFalcon1, climberFalcon2;
  //private DoubleSolenoid climberBrake;
  private DoubleSolenoid climberHookPiston1;
  private DoubleSolenoid climberHookPiston2;
  private DoubleSolenoid climberLeanPiston;

  public ClimberSubsystem() {
    climberFalcon1 = new WPI_TalonFX(ClimberConstants.CLIMBER_1);
    climberFalcon2 = new WPI_TalonFX(ClimberConstants.CLIMBER_2);
    //climberBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_BRAKE_SOLENOID[0], ClimberConstants.CLIMBER_BRAKE_SOLENOID[1]);
    climberHookPiston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_HOOK_PISTONS[0], ClimberConstants.CLIMBER_HOOK_PISTONS[1]);
    climberHookPiston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_HOOK_PISTONS[0], ClimberConstants.CLIMBER_HOOK_PISTONS[1]);
    climberLeanPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_LEAN_PISTON[0], ClimberConstants.CLIMBER_LEAN_PISTON[1]);


    //climberBrake.set(Value.kReverse); // set value for toggling; assume reverse position on startup
    climberHookPiston1.set(Value.kReverse);
    climberHookPiston2.set(Value.kReverse);
    climberLeanPiston.set(Value.kReverse);

    climberFalcon2.follow(climberFalcon1);

    climberFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberFalcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    climberFalcon1.setNeutralMode(NeutralMode.Brake);
    climberFalcon2.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbUp() {
    climberFalcon1.set(ControlMode.PercentOutput, ClimberConstants.CLIMB_SPEED);
  }
  public void climbDown(){
    climberFalcon1.set(ControlMode.PercentOutput, -ClimberConstants.CLIMB_SPEED);
  }

  public void stopClimb(){
      climberFalcon1.set(ControlMode.PercentOutput, 0.0);
  }

  public void toggleBrake(){
    climberFalcon1.set(ControlMode.PercentOutput, 0.1);
  }

  public void toggleHookPistons(){
    climberHookPiston1.toggle();
    climberHookPiston2.toggle();
  }

  public void toggleLeanPistons(){
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

  // public DoubleSolenoid.Value getBrakePos(){
  //   return climberBrake.get(); // returns kOff, kForward, or kReverse
  // }

  // public DoubleSolenoid.Value getPistonsPos(){
  //   return climberHookPiston1.get(); 
  // }
}
