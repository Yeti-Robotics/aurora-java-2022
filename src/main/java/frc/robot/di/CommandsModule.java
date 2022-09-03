package frc.robot.di;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.commands.commandgroups.AllInCommand;
import frc.robot.commands.commandgroups.AllInCommandGroup;
import frc.robot.commands.commandgroups.AllOutCommand;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnToTargetDriveCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ToggleFlywheelHighCommand;
import frc.robot.commands.turret.TurretLockCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoHelper;
import frc.robot.utils.PhotonVision;

import javax.inject.Named;

@Module
public class CommandsModule {
  @Provides
  public TurretLockCommand provideTurretLockCommand(@Named("turret") Subsystem turretSubsystem) {
    return new TurretLockCommand((TurretSubsystem) turretSubsystem);
  }

  @Provides
  public ShooterLEDCommand provideShooterLEDCommand(@Named("led") Subsystem ledSubsystem) {
    return new ShooterLEDCommand((LEDSubsystem) ledSubsystem);
  }

  @Provides
  public ToggleIntakeCommand provideToggleIntakeCommand(@Named("intake") Subsystem intakeSubsystem) {
    return new ToggleIntakeCommand((IntakeSubsystem) intakeSubsystem);
  }

  @Provides
  @Named("two ball")
  public Command provideTwoBall(
          @Named("intake") Subsystem intake,
          @Named("neck") Subsystem neck,
          ShooterLEDCommand shooterLEDCommand,
          @Named("drivetrain") Subsystem drivetrain
  ) {
    IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
    NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
    DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

    return AutoHelper.buildAutonomousCommand(new Command[]{
            new ToggleIntakeCommand(intakeSubsystem),
            new AllInCommand(intakeSubsystem,neckSubsystem)
                    .withTimeout(3.0),
            new WaitCommand(3.0),
            new ToggleIntakeCommand(intakeSubsystem),
            new WaitCommand(1.0),
            new AllInCommand(intakeSubsystem,neckSubsystem)
                    .withTimeout(1.0),
            new InstantCommand(()->ShooterSubsystem.setPoint=4400.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.5),
            new AllInCommand(intakeSubsystem,neckSubsystem)
                    .withTimeout(0.3),
            new WaitCommand(0.5),
            new AllInCommand(intakeSubsystem,neckSubsystem)
                    .withTimeout(0.7),
            new ToggleFlywheelHighCommand(shooterLEDCommand)
    }, new Command[] {
            AutoHelper.runTrajectoryPath(
                    drivetrainSubsystem,
                    Constants.AutoConstants.twoBallPrimary1
            )
    });
  }

  @Provides
  @Named("two ball alt")
  public Command provideTwoBallAlt(
          @Named("intake") Subsystem intake,
          @Named("neck") Subsystem neck,
          ShooterLEDCommand shooterLEDCommand,
          @Named("drivetrain") Subsystem drivetrain
  ) {
    IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
    NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
    DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

    return AutoHelper.buildAutonomousCommand(new Command[]{
            new ToggleIntakeCommand(intakeSubsystem),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(3.0),
            new ToggleIntakeCommand(intakeSubsystem),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(3.0),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(2.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand)
    }, new Command[] {
            AutoHelper.runTrajectoryPath(
                    drivetrainSubsystem,
                    Constants.AutoConstants.twoBallAlternative
            ),
            new TurnToTargetDriveCommand(drivetrainSubsystem)
                    .withTimeout(3.0)
    });
  }

  @Provides
  @Named("one ball")
  public Command provideOneBall(
          @Named("intake") Subsystem intake,
          @Named("neck") Subsystem neck,
          ShooterLEDCommand shooterLEDCommand,
          @Named("drivetrain") Subsystem drivetrain
  ) {
    IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
    NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
    DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

    return AutoHelper.buildAutonomousCommand(new Command[]{
            new DriveForDistanceCommand(drivetrainSubsystem, 48.0, -0.2),
            new InstantCommand(
                    () -> ShooterSubsystem.setPoint = ((25 / 3.0) * PhotonVision.getDistance()) + 2991.66667),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.0),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(3.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand)
    }, new Command[] {
    });
  }

  @Provides
  @Named("four ball")
  public Command provideFourBall(
    @Named("intake") Subsystem intake,
    @Named("neck") Subsystem neck,
    ShooterLEDCommand shooterLEDCommand,
    @Named("drivetrain") Subsystem drivetrain
  ) {
      IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
      NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
      DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

      return AutoHelper.buildAutonomousCommand(new Command[]{
              new ToggleIntakeCommand(intakeSubsystem),
              // new InstantCommand(() -> TurretConstants.TURRET_OFFSET = 0.0),
              new AllInCommand(intakeSubsystem, neckSubsystem)
                      .withTimeout(1.5),
              new InstantCommand(() -> ShooterSubsystem.setPoint = 4400.0),
              new InstantCommand(() -> ShooterSubsystem.isShooting = true),
              new WaitCommand(0.75),
              new ToggleIntakeCommand(intakeSubsystem),
              new WaitCommand(0.5),
              new AllInCommand(intakeSubsystem, neckSubsystem)
                      .withTimeout(0.3),
              new WaitCommand(0.5),
              new AllInCommand(intakeSubsystem, neckSubsystem)
                      .withTimeout(0.7),
              new InstantCommand(() -> ShooterSubsystem.isShooting = false),
              new ToggleIntakeCommand(intakeSubsystem),
              new AllInCommand(intakeSubsystem, neckSubsystem, 0.375)
                      .withTimeout(4.25),
              new WaitCommand(2.5),
              new InstantCommand(() -> ShooterSubsystem.setPoint = 4200.0),
              new InstantCommand(() -> ShooterSubsystem.isShooting = true),
              new WaitCommand(0.75),
              new ToggleIntakeCommand(intakeSubsystem),
              new WaitCommand(0.25),
              new AllInCommand(intakeSubsystem, neckSubsystem)
                      .withTimeout(0.4),
              new WaitCommand(0.5),
              new AllInCommand(intakeSubsystem, neckSubsystem)
                      .withTimeout(0.7),
              new InstantCommand(() -> ShooterSubsystem.isShooting = false),
              new InstantCommand(() -> Constants.TurretConstants.TURRET_OFFSET = 8.0)
      }, new Command[] {
              AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.fourBall1),
              new WaitCommand(2.25),
              AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.fourBall2)
      });
  }

  @Provides
  @Named("three ball")
  public Command provideThreeBall(
          @Named("intake") Subsystem intake,
          @Named("neck") Subsystem neck,
          ShooterLEDCommand shooterLEDCommand,
          @Named("drivetrain") Subsystem drivetrain
  ) {
    IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
    NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
    DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

    return AutoHelper.buildAutonomousCommand(new Command[]{
            new ToggleIntakeCommand(intakeSubsystem),
            new AllInCommandGroup(intakeSubsystem, neckSubsystem)
                    .withTimeout(2.0),
            new ToggleIntakeCommand(intakeSubsystem),
            new InstantCommand(() -> ShooterSubsystem.setPoint = 3800.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.25),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(0.5),
            new WaitCommand(0.25),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(0.75),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.0),
            new ToggleIntakeCommand(intakeSubsystem),
            new AllInCommandGroup(intakeSubsystem, neckSubsystem)
                    .withTimeout(3.0),
            new InstantCommand(() -> ShooterSubsystem.setPoint = 3800.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.25),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(0.5),
            new WaitCommand(0.25),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(0.75),
            new ToggleFlywheelHighCommand(shooterLEDCommand)
    }, new Command[] {
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.threeBall1),
            new WaitCommand(2.0),
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.threeBall2)
    });
  }

  @Provides
  @Named("two ball dump")
  public Command provideTwoBallDump(
          @Named("intake") Subsystem intake,
          @Named("neck") Subsystem neck,
          ShooterLEDCommand shooterLEDCommand,
          @Named("drivetrain") Subsystem drivetrain
  ) {
    IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
    NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
    DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

    return AutoHelper.buildAutonomousCommand(new Command[]{
            new ToggleIntakeCommand(intakeSubsystem),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(3.0),
            new WaitCommand(3.0),
            new ToggleIntakeCommand(intakeSubsystem),
            new WaitCommand(1.0),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(1.0),
            new InstantCommand(() -> ShooterSubsystem.setPoint = 3600.0),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new WaitCommand(1.5),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(0.3),
            new WaitCommand(0.5),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(0.7),
            new ToggleFlywheelHighCommand(shooterLEDCommand),
            new ToggleIntakeCommand(intakeSubsystem),
            new AllInCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(4),
            new WaitCommand(2),
            new ToggleIntakeCommand(intakeSubsystem),
            new AllOutCommand(intakeSubsystem, neckSubsystem)
                    .withTimeout(1.5)
    }, new Command[] {
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.twoBallPrimary1),
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.twoBallPrimary2),
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.twoBallDump)
    });
  }

  @Provides
  @Named("test auto")
  public Command provideTestAuto(
          @Named("intake") Subsystem intake,
          @Named("neck") Subsystem neck,
          ShooterLEDCommand shooterLEDCommand,
          @Named("drivetrain") Subsystem drivetrain
  ) {
    IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intake;
    NeckSubsystem neckSubsystem = (NeckSubsystem) neck;
    DrivetrainSubsystem drivetrainSubsystem = (DrivetrainSubsystem) drivetrain;

    return AutoHelper.buildAutonomousCommand(new Command[]{
            // new ToggleIntakeCommand(intakeSubsystem),
            // new AllInCommand(intakeSubsystem,
            // neckSubsystem).withTimeout(1.5),
            // new InstantCommand(() -> ShooterSubsystem.setPoint = 4200.0),
            // new ToggleFlywheelHighCommand(shooterLEDCommand),
            // new WaitCommand(0.75),
            // new ToggleIntakeCommand(intakeSubsystem),
            // new WaitCommand(1.25),
            // new AllInCommand(intakeSubsystem,
            // neckSubsystem).withTimeout(0.5),
            // new WaitCommand(0.25),
            // new AllInCommand(intakeSubsystem,
            // neckSubsystem).withTimeout(0.75),
            // new ToggleFlywheelHighCommand(shooterLEDCommand),
            // new ToggleIntakeCommand(intakeSubsystem),
            // new AllInCommand(intakeSubsystem,
            // neckSubsystem).withTimeout(4.5),
            // new ToggleIntakeCommand(intakeSubsystem),
            // new WaitCommand(2.0),
            // new InstantCommand(() -> ShooterSubsystem.setPoint = 4200.0),
            // new ToggleFlywheelHighCommand(shooterLEDCommand),
            // new WaitCommand(1.5),
            // new AllInCommand(intakeSubsystem,
            // neckSubsystem).withTimeout(0.5),
            // new WaitCommand(0.25),
            // new AllInCommand(intakeSubsystem,
            // neckSubsystem).withTimeout(0.5),
            // new ToggleFlywheelHighCommand(shooterLEDCommand)
    }, new Command[] {
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.fourBall1),
            new WaitCommand(2.25),
            AutoHelper.runTrajectoryPath(drivetrainSubsystem, Constants.AutoConstants.fourBall2)
    });
  }
}
