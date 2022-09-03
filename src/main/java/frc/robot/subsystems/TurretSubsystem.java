// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.Limelight;
import javax.inject.Inject;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax turretSpark;
  private final DigitalInput magSwitch;
  private final RelativeEncoder turretEncoder;

  public enum TurretLockStatus {
    LOCKED,
    UNLOCKED
  }

  public TurretLockStatus lockStatus;

  @Inject
  public TurretSubsystem(CANSparkMax turret, DigitalInput magSwitch) {
    turretSpark = turret;
    this.magSwitch = magSwitch;
    turretEncoder = turretSpark.getEncoder();
    lockStatus = TurretLockStatus.UNLOCKED;
  }

  @Override
  public void periodic() {}

  public void moveTurret(double power) {
    turretSpark.set(power);
  }

  public void stopTurret() {
    turretSpark.set(0.0);
  }

  public void resetEncoder() {
    turretEncoder.setPosition(0.0);
  }

  public double getEncoder() {
    return turretEncoder.getPosition();
  }

  public boolean getMagSwitch() {
    return !magSwitch.get();
  }

  public double getTurretOffset() {
    return Math.toDegrees(
        Math.atan(TurretConstants.TURRET_OFFSET / (Limelight.getDistance() + 24.0)));
  }
}
