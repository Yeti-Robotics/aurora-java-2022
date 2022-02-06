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
  private WPI_TalonFX climberRightFalcon, climberLeftFalcon;
  private DoubleSolenoid climberBrake;
  private DoubleSolenoid climberPistons;

  public ClimberSubsystem() {
    climberRightFalcon = new WPI_TalonFX(ClimberConstants.RIGHT_CLIMBER);
    climberLeftFalcon = new WPI_TalonFX(ClimberConstants.LEFT_CLIMBER);
    climberBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_BRAKE_SOLENOID[0], ClimberConstants.CLIMBER_BRAKE_SOLENOID[1]);
    climberPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLIMBER_PISTONS[0], ClimberConstants.CLIMBER_PISTONS[1]);

    climberBrake.set(Value.kReverse); // set value for toggling; assume reverse position on startup
    climberPistons.set(Value.kReverse);

    climberLeftFalcon.follow(climberRightFalcon);

    climberRightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberLeftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    climberRightFalcon.setNeutralMode(NeutralMode.Brake);
    climberLeftFalcon.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbUp() {
    climberRightFalcon.set(ControlMode.PercentOutput, ClimberConstants.CLIMB_SPEED);
  }
  public void climbDown(){
    climberRightFalcon.set(ControlMode.PercentOutput, -ClimberConstants.CLIMB_SPEED);
  }

  public void stopClimb(){
      climberRightFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  public void toggleBrake(){
    climberBrake.toggle();
  }

  public void togglePistons(){
    climberPistons.toggle();
  }

  public double getLeftEncoder(){
    return climberLeftFalcon.getSelectedSensorPosition();
  }

  public double getRightEncoder(){
    return climberRightFalcon.getSelectedSensorPosition();
  }

  public double getAverageEncoder(){
    return (getLeftEncoder() + getRightEncoder()) / 2.0;
  }

  public void resetEncoders(){
    climberLeftFalcon.setSelectedSensorPosition(0.0);
    climberRightFalcon.setSelectedSensorPosition(0.0);
  }

  public DoubleSolenoid.Value getBrakePos(){
    return climberBrake.get(); // returns kOff, kForward, or kReverse
  }

  public DoubleSolenoid.Value getPistonsPos(){
    return climberPistons.get(); 
  }
}
