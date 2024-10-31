package frc.robot.Commands.SubCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Pivot;
import frc.robot.Subsytems.Vision;

public class AdjustPivotCommand extends Command {
    private final Pivot pivot;
    private final Vision vision;
    private final String targetName;
    private final Double fixedAngle; // If provided, pivot to this angle instead of using vision data
    private double targetPosition;

    public AdjustPivotCommand(Pivot pivot, Vision vision, String targetName) {
        this.pivot = pivot;
        this.vision = vision;
        this.targetName = targetName;
        this.fixedAngle = null;
        addRequirements(pivot);
    }

    public AdjustPivotCommand(Pivot pivot, double angle) {
        this.pivot = pivot;
        this.vision = null;
        this.targetName = null;
        this.fixedAngle = angle;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        if (fixedAngle != null) {
            targetPosition = fixedAngle;
            System.out.println("[AdjustPivotCommand] Initializing to fixed angle: " + fixedAngle);
        } else {
            System.out.println("[AdjustPivotCommand] Initializing using vision data for target: " + targetName);
            double ty = vision.getTY(targetName);
            targetPosition = ty;
        }
        pivot.rotateToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return pivot.isAtPositionSetpoint(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("[AdjustPivotCommand] Interrupted.");
        } else {
            System.out.println("[AdjustPivotCommand] Completed.");
        }
    }
}
