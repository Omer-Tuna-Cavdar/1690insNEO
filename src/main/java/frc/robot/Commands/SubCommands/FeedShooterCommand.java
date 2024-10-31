package frc.robot.Commands.SubCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Intake;
import frc.robot.Subsytems.Shooter;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class FeedShooterCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;
    private final Timer timer = new Timer();

    public FeedShooterCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        System.out.println("[FeedShooterCommand] Initialized.");
        timer.reset();
        timer.start();
        intake.runIntake(Constants.IntakeConstants.Intake_Roller_Speed);
    }

    @Override
    public void execute() {
        // No additional actions needed during execution
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= Constants.ShooterConstans.FEED_DURATION_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeRollers();
        shooter.stopShooter();
        if (interrupted) {
            System.out.println("[FeedShooterCommand] Interrupted.");
        } else {
            System.out.println("[FeedShooterCommand] Completed.");
        }
    }
}
