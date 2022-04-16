package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.PhotonVision;
import java.util.ArrayList;
import java.util.Arrays;

public class ShooterLEDCommand extends CommandBase {

  private final LEDSubsystem ledSubsystem;
  private final int[] red = {255, 0, 0};
  private final int[] green = {0, 255, 0};
  private final int[] white = {255, 255, 255};
  private ArrayList<int[]> colorQueue;

  public ShooterLEDCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // colorQueue = new int[ledSubsystem.getBufferLength()][3];
    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    int[][] initialState = new int[ledSubsystem.getBufferLength()][3];
    int[] currRGB = green.clone();

    int end = (int) Math.ceil(ledSubsystem.getBufferLength() / 2.0);
    int[] increments = {
      (white[0] - green[0]) / end, (white[1] - green[1]) / end, (white[2] - green[2]) / end
    };

    for (int i = 0; i < end; i++) {
      initialState[i] = new int[] {currRGB[0], currRGB[1], currRGB[2]};

      for (int j = 0; j < 3; j++) {
        currRGB[j] += increments[j];
      }
    }

    int idx = (ledSubsystem.getBufferLength() / 2) - 1;
    for (int i = end; i < ledSubsystem.getBufferLength(); i++) {
      initialState[i] =
          new int[] {initialState[idx][0], initialState[idx][1], initialState[idx][2]};
      idx--;
    }

    colorQueue = new ArrayList<int[]>(Arrays.asList(initialState));
  }

  @Override
  public void execute() {
    if (ShooterSubsystem.atSetPoint
        && Math.abs(PhotonVision.getDistance() - ShooterConstants.SHOOTER_HIGH_DIST)
            <= ShooterConstants.SHOOTER_DIST_TOLERANCE) {
      // wave effect
      for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
        ledSubsystem.setRGB(i, colorQueue.get(i)[0], colorQueue.get(i)[1], colorQueue.get(i)[2]);
      }
      colorQueue.add(0, colorQueue.remove(ledSubsystem.getBufferLength() - 1));
    } else if (ShooterSubsystem.atSetPoint) {
      for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
        ledSubsystem.setRGB(i, green[0], green[1], green[2]);
      }
    } else {
      for (int i = 0; i < ledSubsystem.getBufferLength(); i++) {
        ledSubsystem.setRGB(i, red[0], red[1], red[2]);
      }
    }
    ledSubsystem.sendData();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
