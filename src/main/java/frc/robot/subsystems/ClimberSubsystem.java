// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX climberFalcon1, climberFalcon2;
    private DoubleSolenoid climberBrake;

    public ClimberSubsystem() {
        climberFalcon1 = new WPI_TalonFX(ClimberConstants.CLIMBER_1);
        climberFalcon2 = new WPI_TalonFX(ClimberConstants.CLIMBER_2);

        climberBrake =
                new DoubleSolenoid(
                        PneumaticsModuleType.CTREPCM,
                        ClimberConstants.CLIMBER_BRAKE[0],
                        ClimberConstants.CLIMBER_BRAKE[1]);

        climberBrake.set(Value.kForward);

        climberFalcon1.setInverted(true);
        climberFalcon2.follow(climberFalcon1);
        climberFalcon2.setInverted(InvertType.FollowMaster);

        climberFalcon1.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        climberFalcon2.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        climberFalcon1.enableVoltageCompensation(true);
        climberFalcon2.enableVoltageCompensation(true);

        climberFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        climberFalcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        climberFalcon1.setNeutralMode(NeutralMode.Brake);
        climberFalcon2.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {}

    public void climbUp() {
        if (Value.kForward == climberBrake.get()
                && getAverageEncoder() <= ClimberConstants.CLIMBER_UPRIGHT_HEIGHT_LIMIT) {
            climberFalcon1.set(ControlMode.PercentOutput, ClimberConstants.CLIMB_SPEED);
        } else {
            stopClimb();
        }
    }

    public void climbDown() {
        if (Value.kForward == climberBrake.get()
                && getAverageEncoder() >= ClimberConstants.CLIMBER_TOLERANCE) {
            climberFalcon1.set(ControlMode.PercentOutput, -ClimberConstants.CLIMB_SPEED);
        } else {
            stopClimb();
        }
    }

    public void stopClimb() {
        climberFalcon1.set(ControlMode.PercentOutput, 0.0);
    }

    public double getLeftEncoder() {
        return climberFalcon2.getSelectedSensorPosition();
    }

    public double getRightEncoder() {
        return climberFalcon1.getSelectedSensorPosition();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public void resetEncoders() {
        climberFalcon2.setSelectedSensorPosition(0.0);
        climberFalcon1.setSelectedSensorPosition(0.0);
    }

    public boolean atEncoderLimit() {
        return getAverageEncoder() <= ClimberConstants.CLIMBER_TOLERANCE
                || getAverageEncoder() + ClimberConstants.CLIMBER_TOLERANCE
                        >= ClimberConstants.CLIMBER_UPRIGHT_HEIGHT_LIMIT;
    }

    public void toggleClimberBrake() {
        climberBrake.toggle();
    }
}
