package frc.robot.Commands.SubCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.SwerveSubsystem;
import frc.robot.Subsytems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurnToTargetCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    private final String targetName;
    private Pose2d targetPose;

    public TurnToTargetCommand(SwerveSubsystem swerveSubsystem, Vision vision, String targetName) {
        this.swerveSubsystem = swerveSubsystem;
        this.vision = vision;
        this.targetName = targetName;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("[TurnToTargetCommand] Initializing for target: " + targetName);
    }

    @Override
    public void execute() {
        targetPose = vision.getTargetPose(targetName);
        if (targetPose == null) {
            System.out.println("[TurnToTargetCommand] No pose detected for target: " + targetName);
            return;
        }

        Pose2d robotPose = swerveSubsystem.getPose();
        Rotation2d desiredYaw = calculateDesiredYaw(robotPose, targetPose);

        swerveSubsystem.drive(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            calculateYawRate(robotPose.getRotation(), desiredYaw),
            false
        );
    }

    @Override
    public boolean isFinished() {
        targetPose = vision.getTargetPose(targetName);
        if (targetPose == null) {
            return true;
        }

        Pose2d robotPose = swerveSubsystem.getPose();
        Rotation2d desiredYaw = calculateDesiredYaw(robotPose, targetPose);

        double yawError = Math.abs(desiredYaw.minus(robotPose.getRotation()).getDegrees());
        double rotationalVelocity = swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond;

        return yawError < 3.0 && Math.abs(rotationalVelocity) < Math.toRadians(5);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            0.0,
            false
        );
        if (interrupted) {
            System.out.println("[TurnToTargetCommand] Interrupted.");
        } else {
            System.out.println("[TurnToTargetCommand] Completed for target: " + targetName);
        }
    }

    private Rotation2d calculateDesiredYaw(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();
        double angleRadians = Math.atan2(deltaY, deltaX);
        return new Rotation2d(angleRadians);
    }

    private double calculateYawRate(Rotation2d currentYaw, Rotation2d desiredYaw) {
        double yawErrorDegrees = desiredYaw.minus(currentYaw).getDegrees();
        double kP = 0.02;
        double rotationSpeed = kP * yawErrorDegrees;
        double maxRotationSpeed = Math.toRadians(180);
        return Math.max(-maxRotationSpeed, Math.min(rotationSpeed, maxRotationSpeed));
    }
}
