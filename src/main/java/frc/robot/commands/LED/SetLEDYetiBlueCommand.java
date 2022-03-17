// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDYetiBlueCommand extends CommandBase {
  private final LEDSubsystem ledSubsystem;
  public SetLEDYetiBlueCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem; 
    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("SetLEDYetiBlueCommand.initialize");
    for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
      ledSubsystem.setRGB(i, 20, 120, 255);
    }
    ledSubsystem.sendData();
  }

  @Override
  public void execute() {
    ledSubsystem.sendData();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
