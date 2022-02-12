// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDYetiBlueCommand extends CommandBase {
  /** Creates a new SetLEDYetiBlueCommand. */
  private final LEDSubsystem ledSubsystem;
  public SetLEDYetiBlueCommand(LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem; 
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetLEDYetiBlueCommand.initialize");
    for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
      ledSubsystem.setRGB(i, 20, 120, 255);
      // i dont know if this is the correct value
    }
    ledSubsystem.sendData();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSubsystem.sendData();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
