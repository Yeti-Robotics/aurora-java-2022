// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class BlinkLEDCommand extends CommandBase {
  LEDSubsystem ledSubsystem;
  long commandScheduleTime;
  long startTime;
  int waitTime;
  int r, g, b;
  boolean on = true;

  /**
   * Dont use this
   */
  public BlinkLEDCommand() {}

  public BlinkLEDCommand(LEDSubsystem ledSubsystem, int waitTime, int r, int g, int b) {
    this.ledSubsystem = ledSubsystem;
    this.startTime = System.currentTimeMillis();
    this.waitTime = waitTime;
    this.r = r;
    this.g = g;
    this.b = b;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
      ledSubsystem.setRGB(i, r, g, b);
    }
    ledSubsystem.sendData();
    commandScheduleTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() - startTime >= waitTime) {
      if (on) {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
          ledSubsystem.setRGB(i, 0, 0, 0);
        }
        ledSubsystem.sendData();
        on = false;
      } else {
        for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
          ledSubsystem.setRGB(i, r, g, b);
        }
        ledSubsystem.sendData();
        on = true;
      }
      startTime = System.currentTimeMillis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandScheduleTime + 5000 <= System.currentTimeMillis();
  }
}
