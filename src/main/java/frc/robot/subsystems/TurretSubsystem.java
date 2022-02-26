// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
	private CANSparkMax turretSpark;
	private DigitalInput magSwitch;
    private RelativeEncoder turretEncoder;
    private double power;

    public enum TurretLockStatus {
        LOCKED, UNLOCKED
    }

    public TurretLockStatus lockStatus;

	public TurretSubsystem() {
		turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
		magSwitch = new DigitalInput(TurretConstants.MAG_SWITCH_PORT);
        turretEncoder = turretSpark.getEncoder();
        lockStatus = TurretLockStatus.LOCKED;
	}

    @Override
    public void periodic() {
        if (power > 0 && isRightLimit() || power < 0 && isLeftLimit()) {
            stopTurret();
        }
        // System.out.println("TURRET ENC: " + turretPosition);
    }

	public void moveTurret(double power) {
        turretSpark.set(this.power = power);
    }
    
    public void stopTurret() {
        turretSpark.set(power = 0.0);
    }

    public boolean isLeftLimit(){
        double turretPosition = getEncoder();
        return turretPosition <= TurretConstants.TURRET_MAX_LEFT + TurretConstants.TURRET_TOLERANCE;
    }

    public boolean isRightLimit(){
        double turretPosition = getEncoder();
        return turretPosition >= TurretConstants.TURRET_MAX_RIGHT - TurretConstants.TURRET_TOLERANCE;
    }

    public boolean isWithinRotationLimit() {
        double turretPosition = getEncoder();
        return turretPosition <= TurretConstants.TURRET_MAX_RIGHT - TurretConstants.TURRET_TOLERANCE &&
				turretPosition >= TurretConstants.TURRET_MAX_LEFT + TurretConstants.TURRET_TOLERANCE;
    }

    public void resetEncoder(){
        turretEncoder.setPosition(0.0);
    }

    public double getEncoder(){
        return turretEncoder.getPosition();
    }

    public boolean getMagSwitch(){
        return !magSwitch.get();
    }

    public double getPower(){
        return power;
    }
}
