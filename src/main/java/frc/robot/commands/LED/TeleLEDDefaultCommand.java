// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.PhotonVision;

public class TeleLEDDefaultCommand extends CommandBase {

  private final LEDSubsystem ledSubsystem;
  private int[] currColor = { 0, 0, 0 };
  private final int[] white = { 255, 255, 255 };
  private final int[] blue = { 20, 120, 255 };

  public TeleLEDDefaultCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math
        .abs(Limelight.getDistance() - ShooterConstants.SHOOTER_HIGH_DIST) <= ShooterConstants.SHOOTER_DIST_TOLERANCE) {
      currColor = white;
    } else {
      currColor = blue;
    }

    for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
      ledSubsystem.setRGB(i, currColor[0], currColor[1], currColor[2]);
    }

    ledSubsystem.sendData();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
