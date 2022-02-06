// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import frc.robot.Constants.TurretConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
	private CANSparkMax turretSpark;
	private SparkMaxLimitSwitch magSwitch;
    private RelativeEncoder turretEncoder;

	public TurretSubsystem() {
		turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
		magSwitch = turretSpark.getForwardLimitSwitch(Type.kNormallyClosed);
        turretEncoder = turretSpark.getEncoder();
	}

	public void moveTurret(double power) {
        turretSpark.set(power);
    }
    
    public void stopTurret() {
        turretSpark.set(0.0);
    }

    public void resetEncoder(){
        turretEncoder.setPosition(0.0);
    }

    public double getEncoder(){
        return turretEncoder.getPosition();
    }

    public boolean getMagSwitch(){
        return magSwitch.isPressed();
    }
}
