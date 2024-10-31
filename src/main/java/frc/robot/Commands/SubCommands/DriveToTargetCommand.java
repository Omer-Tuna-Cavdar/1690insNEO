package frc.robot.Commands.SubCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.SwerveSubsystem;
import frc.robot.Subsytems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveToTargetCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    private final String targetName;
    private final double targetDistance; // Desired distance from the target

    public DriveToTargetCommand(SwerveSubsystem swerveSubsystem, Vision vision, String targetName, double targetDistance) {
        this.swerveSubsystem = swerveSubsystem;
        this.vision = vision;
        this.targetName = targetName;
        this.targetDistance = targetDistance;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("[DriveToTargetCommand] Initializing for target: " + targetName);
    }

    @Override
    public void execute() {
        Pose2d targetPose = vision.getTargetPose(targetName);
        if (targetPose == null) {
            System.out.println("[DriveToTargetCommand] Target pose not available for: " + targetName);
            swerveSubsystem.drive(
                new Translation2d(0, 0),
                0.0,
                false
            );
            return;
        }

        Pose2d robotPose = swerveSubsystem.getPose();
        Translation2d relativeTranslation = targetPose.getTranslation().minus(robotPose.getTranslation());

        // Determine chassis speeds
        double xSpeed = relativeTranslation.getX() * 0.5; // Scale down for control
        double ySpeed = relativeTranslation.getY() * 0.5;

        // Calculate desired rotation to face target
        double desiredYaw = Math.atan2(relativeTranslation.getY(), relativeTranslation.getX());
        Rotation2d desiredRotation = new Rotation2d(desiredYaw);
        double yawError = desiredRotation.minus(robotPose.getRotation()).getRadians();

        // Simple proportional controller for rotation
        double kP = 0.5; // Proportional gain (tune as needed)
        double rotSpeed = kP * yawError;

        // Clamp rotational speed
        double maxRotSpeed = Math.toRadians(180); // 180 deg/s
        rotSpeed = Math.max(-maxRotSpeed, Math.min(rotSpeed, maxRotSpeed));

        // Command the swerve drive
        swerveSubsystem.drive(
            new Translation2d(xSpeed, ySpeed),
            rotSpeed,
            true // Field-relative
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = vision.getTargetPose(targetName);
        if (targetPose == null) {
            return false;
        }
        Pose2d robotPose = swerveSubsystem.getPose();
        double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        return distanceToTarget < targetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(
            new Translation2d(0, 0),
            0.0,
            false
        );
        if (interrupted) {
            System.out.println("[DriveToTargetCommand] Interrupted.");
        } else {
            System.out.println("[DriveToTargetCommand] Completed for target: " + targetName);
        }
    }
}
