package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForDistanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double distanceGoal;
    private double power;

    public DriveForDistanceCommand(
            DrivetrainSubsystem drivetrainSubsystem, double encoderGoal, double power) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.distanceGoal = encoderGoal;
        this.power = Math.abs(power);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.resetEncoders();
    }

    @Override
    public void execute() {
        if (distanceGoal < 0) {
            drivetrainSubsystem.cheezyDrive(-power, 0.0);
        } else {
            drivetrainSubsystem.cheezyDrive(power, 0.0);
        }
        System.out.println(drivetrainSubsystem.getAverageEncoder() + "; " + distanceGoal);
    }

    @Override
    public boolean isFinished() {
        return (distanceGoal < 0)
                ? drivetrainSubsystem.getAverageEncoder() <= distanceGoal
                : drivetrainSubsystem.getAverageEncoder() >= distanceGoal;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("stop");
        drivetrainSubsystem.stopDrive();
    }
}
