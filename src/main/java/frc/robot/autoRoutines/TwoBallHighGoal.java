package frc.robot.autoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TwoBallHighGoal extends SequentialCommandGroup {

    public TwoBallHighGoal (double drivePower, DrivetrainSubsystem drivetrainSubsystem) { //add shooter and intake parameters

        addCommands(
            //shoot preloaded ball
            new DriveForDistanceCommand(drivetrainSubsystem, -10.0, drivePower, drivePower).withTimeout(2)
            //intake and shoot next ball
        );
    }
}