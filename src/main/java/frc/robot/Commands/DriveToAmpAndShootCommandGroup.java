package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.*;
import frc.robot.Constants;
import frc.robot.Commands.SubCommands.*;

public class DriveToAmpAndShootCommandGroup extends SequentialCommandGroup {

    public DriveToAmpAndShootCommandGroup(SwerveSubsystem swerveSubsystem, Vision vision, Shooter shooter, Pivot pivot, Intake intake) {

        addCommands(
            new TurnToTargetCommand(swerveSubsystem, vision, "Amp"),
            new ParallelCommandGroup(
                new DriveToTargetCommand(swerveSubsystem, vision, "Amp", 0.5),
                new AdjustPivotCommand(pivot, 75), 
                new SpinUpShooterCommand(shooter, Constants.ShooterConstans.SHOOTER_RPM_FORAMP)
            ),
            new ParallelCommandGroup(
                new FeedShooterCommand(intake, shooter),
                new AdjustPivotCommand(pivot, 90)
            )
        );
    }
}
