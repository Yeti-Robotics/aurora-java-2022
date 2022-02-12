// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class AuroraLEDCommand extends CommandBase {
  /** Creates a new AuroraLEDCommand. */
  private final LEDSubsystem ledSubsystem;

  // RGB
  private final int[] mintGreen = {30, 222, 32};
  private final int mintGreenPinkBoundary = LEDConstants.LED_COUNT / 36;

  private final int[] pink = {199, 68, 235};
  private final int pinkLightBlueBoundary = LEDConstants.LED_COUNT / 18;

  private final int[] lightBlue = {4, 255, 219};
  private final int lightBlueDarkBlueBoundary = (3 * LEDConstants.LED_COUNT) / 36;

  private final int[] darkBlue = {62, 0, 216};
  private final int darkBlueMintGreenBoundary = LEDConstants.LED_COUNT / 9;

  private final int gradientLength = 3;
  private final int[][] mintGreenPinkGradient = calcGradientColors(mintGreen, pink);
  private final int[][] pinkLightBlueGradient = calcGradientColors(pink, lightBlue);
  private final int[][] lightBlueDarkBlueGradient = calcGradientColors(lightBlue, darkBlue);
  private final int[][] darkBlueMintGreenGradient = calcGradientColors(darkBlue, mintGreen);

  private int position = 0;

  private long startTime = System.currentTimeMillis();
  private final int waitTime = 50;

  public AuroraLEDCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private int wrapValues(int num) {
    return num % LEDConstants.LED_COUNT;
  }

  private int[] getAvgValue(int[] color1, int[] color2) {
    return new int[] {(color1[0] + color2[0]) / 2, (color1[1] + color2[1]) / 2, (color1[2] + color2[2]) / 2};
  }

  private int[][] calcGradientColors(int[] color1, int[] color2) {
    int[][] gradientColors = new int[gradientLength][3];

    final double centerIndex = (gradientLength - 1.0) / 2.0;

    int[] leftColor = getAvgValue(color1, color2);
    int[] rightColor = getAvgValue(color1, color2);

    // [[], [], [], []]
    // [[], [], [], [], [], [], []]
    // if odd num of gradient leds set the middle to avg of both sides
    if (gradientLength % 2 != 0) gradientColors[(int) centerIndex] = leftColor;

    for (int i = 0; i < Math.floor(gradientLength / 2.0); i++) {
      rightColor = getAvgValue(rightColor, color2);
      leftColor = getAvgValue(color1, leftColor);
      int placementIndex = i + 1;
      gradientColors[(int) Math.floor(centerIndex + placementIndex)] = rightColor;
      gradientColors[(int) Math.ceil(centerIndex - placementIndex)] = leftColor;
    }

    return gradientColors;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() - startTime >= waitTime) {
      for (int j = 0; j < LEDConstants.LED_COUNT + 1; j += (LEDConstants.LED_COUNT / 9)) {
        int i = 0;
        for (i = i + j + position; i < (mintGreenPinkBoundary - gradientLength) + j + position; i++) {
          ledSubsystem.setRGB(wrapValues(i), mintGreen[0], mintGreen[1], mintGreen[2]);
        }
        for (; i < mintGreenPinkBoundary + j + position; i++) {
          // makes i (0 through gradientLength - 1) for selecting each color out of the gradient
          int gradientPos = i - j - position - (mintGreenPinkBoundary - gradientLength);
          ledSubsystem.setRGB(wrapValues(i), mintGreenPinkGradient[gradientPos][0], mintGreenPinkGradient[gradientPos][1], mintGreenPinkGradient[gradientPos][2]);
        }
        for (; i < (pinkLightBlueBoundary - gradientLength) + j + position; i++) {
          ledSubsystem.setRGB(wrapValues(i), pink[0], pink[1], pink[2]);
        }
        for (; i < pinkLightBlueBoundary + j + position; i++) {
          int gradientPos = i - j - position - (pinkLightBlueBoundary - gradientLength);
          ledSubsystem.setRGB(wrapValues(i), pinkLightBlueGradient[gradientPos][0], pinkLightBlueGradient[gradientPos][1], pinkLightBlueGradient[gradientPos][2]);
        }
        for (; i < (lightBlueDarkBlueBoundary - gradientLength) + j + position; i++) {
          ledSubsystem.setRGB(wrapValues(i), lightBlue[0], lightBlue[1], lightBlue[2]);
        }
        for (; i < lightBlueDarkBlueBoundary + j + position; i++) {
          int gradientPos = i - j - position - (lightBlueDarkBlueBoundary - gradientLength);
          ledSubsystem.setRGB(wrapValues(i), lightBlueDarkBlueGradient[gradientPos][0], lightBlueDarkBlueGradient[gradientPos][1], lightBlueDarkBlueGradient[gradientPos][2]);
        }
        for (; i < (darkBlueMintGreenBoundary - gradientLength) + j + position; i++) {
          ledSubsystem.setRGB(wrapValues(i), darkBlue[0], darkBlue[1], darkBlue[2]);
        }
        for (; i < darkBlueMintGreenBoundary + j + position; i++) {
          int gradientPos = i - j - position - (darkBlueMintGreenBoundary - gradientLength);
          ledSubsystem.setRGB(wrapValues(i), darkBlueMintGreenGradient[gradientPos][0], darkBlueMintGreenGradient[gradientPos][1], darkBlueMintGreenGradient[gradientPos][2]);
        }
      }
      ledSubsystem.sendData();
      position = wrapValues(position + 1);
      startTime = System.currentTimeMillis();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
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
