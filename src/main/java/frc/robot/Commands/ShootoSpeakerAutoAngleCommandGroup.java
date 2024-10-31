package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.*;
import frc.robot.Commands.SubCommands.*;

public class ShootoSpeakerAutoAngleCommandGroup extends SequentialCommandGroup {

    public ShootoSpeakerAutoAngleCommandGroup(Pivot pivot, Vision vision, Shooter shooter, Intake intake, SwerveSubsystem swerveSubsystem) {

        addCommands(
            new TurnToTargetCommand(swerveSubsystem, vision, "Speaker"),
            new ParallelCommandGroup(
                new AdjustPivotCommand(pivot, vision, "Speaker"),
                new SpinUpShooterCommand(shooter, vision, "Speaker")
            ),
            new FeedShooterCommand(intake, shooter)
        );
    }
}
