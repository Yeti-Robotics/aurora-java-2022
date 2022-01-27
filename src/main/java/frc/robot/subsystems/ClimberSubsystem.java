// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX rightClimberFalcon, leftClimberFalcon;
  private DoubleSolenoid climberBrake;

  public ClimberSubsystem() {
    rightClimberFalcon = new WPI_TalonFX(ClimberConstants.RIGHT_CLIMBER);
    leftClimberFalcon = new WPI_TalonFX(ClimberConstants.LEFT_CLIMBER);
    climberBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.CLIMBER_BRAKE_SOLENOID[0], ClimberConstants.CLIMBER_BRAKE_SOLENOID[1]);

    leftClimberFalcon.follow(rightClimberFalcon);
    leftClimberFalcon.setInverted(true);

    leftClimberFalcon.setNeutralMode(NeutralMode.Brake);
    rightClimberFalcon.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbUp() {
    rightClimberFalcon.set(ControlMode.PercentOutput, ClimberConstants.CLIMB_SPEED);
  }
  public void climbDown(){
    rightClimberFalcon.set(ControlMode.PercentOutput, -ClimberConstants.CLIMB_SPEED);
}

public void stopClimb(){
    rightClimberFalcon.set(ControlMode.PercentOutput, 0.0);
}

public void toggleBrake(){
  climberBrake.toggle();
}

}
