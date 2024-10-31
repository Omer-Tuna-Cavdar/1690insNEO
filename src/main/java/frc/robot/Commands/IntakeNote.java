package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsytems.Intake;

public class IntakeNote extends Command {
    private final Intake intake;
     public IntakeNote(Intake intake) {
        
        this.intake = intake;
        addRequirements(intake); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.runIntake(Constants.IntakeConstants.Intake_Roller_Speed);
    }

    @Override
    public boolean isFinished() {
        // End the command when the pivot reaches the 30-degree setpoint
        return intake.isIntakeBeamBroken();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeRollers();
    }   
}
