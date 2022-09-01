package frc.robot.di;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import javax.inject.Named;

@Module
public class CommandsModule {
  @Provides
  public TurretLockCommand provideTurretLockCommand(TurretSubsystem turretSubsystem) {
    return new TurretLockCommand(turretSubsystem);
  }

  @Provides
  @Named("tank drive")
  public Command provideTankDrive(
      DrivetrainSubsystem drivetrainSubsystem, RobotContainer robotContainer) {
    return new RunCommand(
        () -> drivetrainSubsystem.tankDrive(robotContainer.getLeftY(), robotContainer.getRightY()),
        drivetrainSubsystem);
  }

  @Provides
  @Named("cheezy drive")
  public Command provideCheezyDrive(
      DrivetrainSubsystem drivetrainSubsystem, RobotContainer robotContainer) {
    return new RunCommand(
        () ->
            drivetrainSubsystem.cheezyDrive(robotContainer.getLeftY(), robotContainer.getRightX()),
        drivetrainSubsystem);
  }

  @Provides
  @Named("arcade drive")
  public Command provideArcadeDrive(
      DrivetrainSubsystem drivetrainSubsystem, RobotContainer robotContainer) {
    return new RunCommand(
        () ->
            drivetrainSubsystem.arcadeDrive(robotContainer.getLeftY(), robotContainer.getRightX()),
        drivetrainSubsystem);
  }
}
