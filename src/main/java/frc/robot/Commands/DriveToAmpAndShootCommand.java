package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.SwerveSubsystem;
import frc.robot.Subsytems.Vision;
import frc.robot.Subsytems.Shooter;
import frc.robot.Constants;
import frc.robot.Subsytems.Intake;
import frc.robot.Subsytems.Pivot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveToAmpAndShootCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    private final Shooter shooter;
    private final Pivot pivot;
    private final Intake intake;

    private Pose2d ampPose;
    private boolean isTargetAcquired = false;
    private boolean isPivotAtTarget = false;
    private boolean isShooterRunning = false;

    public DriveToAmpAndShootCommand(SwerveSubsystem swerveSubsystem, Vision vision, Shooter shooter, Pivot pivot, Intake intake) {
        this.swerveSubsystem = swerveSubsystem;
        this.vision = vision;
        this.shooter = shooter;
        this.pivot = pivot;
        this.intake= intake;
        addRequirements(swerveSubsystem, vision, shooter, pivot,intake);
    }

    @Override
    public void initialize() {
        System.out.println("[DriveToAmpAndShootCommand] Initializing.");
        // Retrieve the amp's pose from Vision subsystem
        ampPose = vision.getAmpPose(); // Ensure this method exists in Vision
        if (ampPose == null) {
            System.out.println("[DriveToAmpAndShootCommand] Amp pose not detected. Command will terminate.");
            cancel();
        } else {
            isTargetAcquired = true;
            System.out.println("[DriveToAmpAndShootCommand] Amp pose acquired: " + ampPose);
            // Set shooter to slow RPM
            intake.runIntake(Constants.IntakeConstants.Intake_Roller_Speed_ForAmp);
            shooter.runShooter(Constants.ShooterConstans.SHOOTER_RPM_FORAMP); // Example slow RPM
            isShooterRunning = true;
            // Set pivot to 90 degrees
            pivot.rotateToPosition(90.0);
        }
    }

    @Override
    public void execute() {
        if (!isTargetAcquired) {
            ampPose = vision.getAmpPose();
            if (ampPose != null) {
                isTargetAcquired = true;
                System.out.println("[DriveToAmpAndShootCommand] Amp pose acquired during execution: " + ampPose);
                // Set shooter to slow RPM
                intake.runIntake(Constants.IntakeConstants.Intake_Roller_Speed_ForAmp);
            shooter.runShooter(Constants.ShooterConstans.SHOOTER_RPM_FORAMP);; // Example slow RPM
                isShooterRunning = true;
                // Set pivot to 90 degrees
                pivot.rotateToPosition(90.0);
            } else {
                // Rotate in place or perform a search pattern if amp is not detected
                swerveSubsystem.drive(
                    new Translation2d(0, 0), // No translation
                    0.2, // Rotate at 0.2 rad/s
                    false // Robot-relative
                );
                return;
            }
        }

        // Drive towards amp
        Pose2d robotPose = swerveSubsystem.getPose();
        Translation2d relativeTranslation = ampPose.getTranslation().minus(robotPose.getTranslation());

        // Determine chassis speeds
        double xSpeed = relativeTranslation.getX() * 0.5; // Scale down for control
        double ySpeed = relativeTranslation.getY() * 0.5;

        // Calculate desired rotation to face amp
        double desiredYaw = Math.atan2(relativeTranslation.getY(), relativeTranslation.getX());
        Rotation2d desiredRotation = new Rotation2d(desiredYaw);
        double currentYaw = robotPose.getRotation().getRadians();
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

        System.out.println("[DriveToAmpAndShootCommand] Driving towards amp. X Speed: " + xSpeed + " m/s, Y Speed: " + ySpeed + " m/s, Rot Speed: " + Math.toDegrees(rotSpeed) + " deg/s");

        // Check if pivot has reached 90 degrees
        if (!isPivotAtTarget && pivot.isAtPositionSetpoint(90.0)) {
            isPivotAtTarget = true;
            System.out.println("[DriveToAmpAndShootCommand] Pivot reached 90 degrees.");
        }

        // Check if shooter is running at slow RPM
        if (isShooterRunning && shooter.isAtTargetRPM(3000)) {
            System.out.println("[DriveToAmpAndShootCommand] Shooter is running at target RPM.");
        }

        // Determine if command should finish
        double distanceToAmp = robotPose.getTranslation().getDistance(ampPose.getTranslation());
        if (distanceToAmp < 0.5 && isPivotAtTarget && isShooterRunning && shooter.isAtTargetRPM(3000)) {
            System.out.println("[DriveToAmpAndShootCommand] All actions completed.");
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        // The command finishes when all actions are complete
        Pose2d robotPose = swerveSubsystem.getPose();
        double distanceToAmp = (ampPose != null) ? robotPose.getTranslation().getDistance(ampPose.getTranslation()) : Double.MAX_VALUE;
        boolean pivotComplete = isPivotAtTarget;
        boolean shooterComplete = isShooterRunning && shooter.isAtTargetRPM(3000);

        return distanceToAmp < 0.5 && pivotComplete && shooterComplete;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(
            new Translation2d(0, 0), // Stop translation
            0.0, // Stop rotation
            false // Robot-relative
        );
        shooter.stopShooter();
        pivot.stow();
        if (interrupted) {
            System.out.println("[DriveToAmpAndShootCommand] Interrupted and ended.");
        } else {
            System.out.println("[DriveToAmpAndShootCommand] Command completed successfully.");
        }
    }

    /**
     * Calculates the desired yaw angle for the robot to face the amp based on the robot's and amp's poses.
     */
    private Rotation2d calculateDesiredYaw(Pose2d robotPose, Pose2d ampPose) {
        // Calculate the relative translation from the robot to the amp
        double deltaX = ampPose.getX() - robotPose.getX();
        double deltaY = ampPose.getY() - robotPose.getY();

        // Calculate the angle from the robot to the amp
        double angleRadians = Math.atan2(deltaY, deltaX);
        Rotation2d desiredYaw = new Rotation2d(angleRadians);

        System.out.println("[DriveToAmpAndShootCommand] Calculated desired yaw: " + desiredYaw.getDegrees() + " degrees.");
        return desiredYaw;
    }

    /**
     * Calculates the rotational rate needed to turn the robot towards the desired yaw.
     * This uses a simple proportional control for demonstration purposes.
     */
    private double calculateYawRate(Rotation2d currentYaw, Rotation2d desiredYaw) {
        double yawErrorDegrees = desiredYaw.minus(currentYaw).getDegrees();
        double kP = 0.02; // Proportional gain (tune as needed)

        // Calculate rotational speed with proportional control
        double rotationSpeed = kP * yawErrorDegrees;

        // Clamp the rotation speed to a maximum value
        double maxRotationSpeed = Math.toRadians(180); // Max 180 degrees per second
        rotationSpeed = Math.max(-maxRotationSpeed, Math.min(rotationSpeed, maxRotationSpeed));

        System.out.println("[DriveToAmpAndShootCommand] Yaw Error: " + yawErrorDegrees + " degrees, Rotation Speed: " + Math.toDegrees(rotationSpeed) + " degrees/sec");
        return rotationSpeed;
    }
}
