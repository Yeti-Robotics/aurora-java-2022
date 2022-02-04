// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import frc.robot.Constants.TurretConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
	CANSparkMax turretSpark;
	SparkMaxAnalogSensor magSwitch;
    RelativeEncoder turretEncoder;

	public TurretSubsystem() {
		turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
		magSwitch = turretSpark.getAnalog(Mode.kAbsolute);
        turretEncoder = turretSpark.getEncoder();
	}

	@Override
	public void periodic() {}

	public void moveTurret(double power) {
        turretSpark.set(power);
    }

    public void stopTurret() {
        turretSpark.set(0);
    }

    public void resetEncoder(){
        turretEncoder.setPosition(0);
    }

    public double getEncoder(){
        return turretEncoder.getPosition();
    }
}
