package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpCommand extends CommandBase {

	private final ClimberSubsystem climberSubsystem;
	private double limit;

	public ClimbUpCommand(ClimberSubsystem climberSubsystem, double limit) {
		this.climberSubsystem = climberSubsystem;
		this.limit = limit;
		addRequirements(climberSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (!(climberSubsystem.getAverageEncoder() >= limit - ClimberConstants.CLIMBER_TOLERANCE)) { // || climberSubsystem.getSolenoidPos() == Value.kForward)){
			climberSubsystem.climbUp();
		}
	}

	@Override
	public boolean isFinished() {
		return climberSubsystem.getAverageEncoder() >= limit - ClimberConstants.CLIMBER_TOLERANCE; // || climberSubsystem.getSolenoidPos() == Value.kForward;
	}

	@Override
	public void end(boolean interrupted) {
		climberSubsystem.stopClimb();
	}
}