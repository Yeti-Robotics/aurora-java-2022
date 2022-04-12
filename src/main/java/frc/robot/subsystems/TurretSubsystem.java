// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.TurretConstants;
import frc.robot.utils.PhotonVision;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
	private CANSparkMax turretSpark;
	private DigitalInput magSwitch;
    private RelativeEncoder turretEncoder;

    public enum TurretLockStatus {
        LOCKED, UNLOCKED
    }

    public TurretLockStatus lockStatus;

	private TurretSubsystem() {
		turretSpark = new CANSparkMax(TurretConstants.TURRET_SPARK, MotorType.kBrushless);
		magSwitch = new DigitalInput(TurretConstants.MAG_SWITCH_PORT);
        turretEncoder = turretSpark.getEncoder();
        lockStatus = TurretLockStatus.UNLOCKED;

        turretSpark.enableSoftLimit(SoftLimitDirection.kForward, true);
        turretSpark.enableSoftLimit(SoftLimitDirection.kReverse, true);
        turretSpark.setSoftLimit(SoftLimitDirection.kForward, (float) TurretConstants.TURRET_MAX_RIGHT);
        turretSpark.setSoftLimit(SoftLimitDirection.kReverse, (float) TurretConstants.TURRET_MAX_LEFT);
    }

    private static final TurretSubsystem instance = new TurretSubsystem();
    public static TurretSubsystem getInstance() {
        return instance;
    }

    @Override
    public void periodic() {}

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
        return !magSwitch.get();
    }

    public double getTurretOffset(){
        return Math.toDegrees(Math.atan(TurretConstants.TURRET_OFFSET / PhotonVision.getDistance()));
    }
}
