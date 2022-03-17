// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
public class RainbowLEDCommand extends CommandBase {
  private final LEDSubsystem ledSubsystem;
  private int rainbowFirstPixelHue;
  public RainbowLEDCommand(LEDSubsystem ledSubsystem, int rainbowFirstPixelHue) {
    this.ledSubsystem = ledSubsystem;
    this.rainbowFirstPixelHue = rainbowFirstPixelHue;
    addRequirements(ledSubsystem);
  }


  @Override
  public void execute() {
    // For every pixel
      for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = (rainbowFirstPixelHue + (i * 180 / ledSubsystem.getBufferLength())) % 180;
        // Set the value
        ledSubsystem.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      rainbowFirstPixelHue += 3;
      // Check bounds
      rainbowFirstPixelHue %= 180;
      ledSubsystem.sendData();
  }
  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
