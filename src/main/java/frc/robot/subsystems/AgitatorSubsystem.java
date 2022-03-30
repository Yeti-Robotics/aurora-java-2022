// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AgitatorConstants;

public class AgitatorSubsystem extends SubsystemBase {
  private TalonSRX agitatorMotor;
  /** Creates a new AgitatorSubsystem. */
  public AgitatorSubsystem() {
 agitatorMotor = new TalonSRX(AgitatorConstants.AGITATOR_MOTOR);
  }

  public void Spin() {
    agitatorMotor.set(ControlMode.PercentOutput, AgitatorConstants.AGITATER_SPEED);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
