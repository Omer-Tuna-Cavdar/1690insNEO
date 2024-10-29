package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.HelperMerhodes.HelperMethodes;
import frc.robot.Subsytems.Intake;
import frc.robot.Subsytems.Pivot;
import frc.robot.Subsytems.Shooter;
import frc.robot.Subsytems.Vision;
import frc.robot.Subsytems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShootoSpeakerAutoAngle extends Command {
    private final Pivot pivot;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final SwerveSubsystem swerveSubsystem;

    private double targetPosition;
    private boolean noteshot = false;

    // Define the states for the state machine
    private enum State {
        TURNING_TO_SPEAKER,
        ADJUSTING_PIVOT,
        SHOOTING,
        FINISHED
    }

    private State currentState = State.TURNING_TO_SPEAKER;

    public ShootoSpeakerAutoAngle(Pivot pivot, Vision vision, Shooter shooter, Intake intake, SwerveSubsystem swerveSubsystem) {
        this.pivot = pivot;
        this.vision = vision;
        this.shooter = shooter;
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem;

        // Declare subsystem dependencies
        addRequirements(pivot, vision, shooter, intake, swerveSubsystem);
    }

    @Override
    public void initialize() {
        currentState = State.TURNING_TO_SPEAKER;
        noteshot = false;

        // Log initialization
        System.out.println("[ShootoSpeakerAutoAngle] Initialized. Starting TURNING_TO_SPEAKER state.");
    }

    @Override
    public void execute() {
        switch (currentState) {
            case TURNING_TO_SPEAKER:
                handleTurningToSpeaker();
                break;

            case ADJUSTING_PIVOT:
                handleAdjustingPivot();
                break;

            case SHOOTING:
                handleShooting();
                break;

            case FINISHED:
                // Do nothing, waiting for the command to end
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return noteshot;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeRollers();
        shooter.stopShooter();
        pivot.stow();
        swerveSubsystem.drive(
            new edu.wpi.first.math.geometry.Translation2d(0, 0), // No translation
            0.0, // No rotation
            false // Robot-relative
        );

        if (interrupted) {
            System.out.println("[ShootoSpeakerAutoAngle] Command interrupted. Cleaning up.");
        } else {
            System.out.println("[ShootoSpeakerAutoAngle] Command completed successfully.");
        }
    }

    /**
     * Handles the TURNING_TO_SPEAKER state.
     */
    private void handleTurningToSpeaker() {
        // Retrieve the speaker's pose from Vision
        Pose2d speakerPose = vision.getLatestPose();

        if (speakerPose == null) {
            System.out.println("[ShootoSpeakerAutoAngle] No AprilTag detected for speaker. Stopping command.");
            noteshot = true; // End the command if no target is found
            return;
        }

        // Get the robot's current pose from the swerve subsystem
        Pose2d robotPose = swerveSubsystem.getPose();

        // Calculate the desired yaw to face the speaker
        Rotation2d desiredYaw = calculateDesiredYaw(robotPose, speakerPose);

        // Command the swerve drive to rotate towards the desired yaw
        swerveSubsystem.drive(
            new edu.wpi.first.math.geometry.Translation2d(0, 0), // No translation
            calculateYawRate(robotPose.getRotation(), desiredYaw), // Rotation rate
            false // Robot-relative
        );

        // Check if the robot is facing the desired yaw within a tolerance and rotational velocity is low
        double yawError = Math.abs(desiredYaw.minus(robotPose.getRotation()).getDegrees());
        double rotationalVelocity = swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond;

        System.out.println("[ShootoSpeakerAutoAngle] TURNING_TO_SPEAKER - Yaw Error: " + yawError + " degrees, Rotational Velocity: " + rotationalVelocity + " rad/s");

        if (yawError < 3.0 && Math.abs(rotationalVelocity) < Math.toRadians(5)) { // 3 degrees tolerance and low rotational velocity
            System.out.println("[ShootoSpeakerAutoAngle] TURNING_TO_SPEAKER completed.");
            currentState = State.ADJUSTING_PIVOT;
        }
    }

    /**
     * Handles the ADJUSTING_PIVOT state.
     */
    private void handleAdjustingPivot() {
        // Set the target position based on the Limelight's vertical offset (ty)
        double ty = vision.getTY("Speaker");
        targetPosition = ty;
        pivot.rotateToPosition(targetPosition);

        // Reset the pivot encoder if the beam break sensor is triggered
        if (pivot.isPivotBeamBroken()) {
            pivot.resetPivotEncoder();
            System.out.println("[ShootoSpeakerAutoAngle] Pivot encoder reset due to beam break.");
        }

        // Check if the pivot has reached the target position
        if (pivot.isAtPositionSetpoint(targetPosition)) {
            System.out.println("[ShootoSpeakerAutoAngle] ADJUSTING_PIVOT completed.");
            currentState = State.SHOOTING;
        }
    }

    /**
     * Handles the SHOOTING state.
     */
    private void handleShooting() {
        // Calculate the target RPM based on the distance to the speaker
        double distance = vision.getDistance("Speaker");
        if (distance < 0.0) {
            System.out.println("[ShootoSpeakerAutoAngle] Invalid distance to speaker. Stopping command.");
            noteshot = true;
            return;
        }

        double targetRPM = calculateTargetRPM(distance);
        shooter.runShooter(targetRPM);

        // Check if the shooter is at target RPM and maintaining it
        if (shooter.isAtTargetRPM(targetRPM)) {
            intake.runIntake(Constants.IntakeConstants.Intake_Roller_Speed);
            System.out.println("[ShootoSpeakerAutoAngle] SHOOTING - Shooter at target RPM. Running intake.");

            // Optionally, verify that the shooter maintains the target RPM for a certain duration
            // Here, we'll assume that maintaining the RPM triggers the condition to finish
            // Alternatively, you can implement a more sophisticated check with a state timer
            noteshot = true;
            System.out.println("[ShootoSpeakerAutoAngle] SHOOTING completed.");
        }
    }

    /**
     * Calculates the desired yaw angle for the robot to face the speaker based on the robot's and speaker's poses.
     *
     * @param robotPose   The current pose of the robot.
     * @param speakerPose The pose of the speaker obtained from Vision's AprilTag reading.
     * @return The desired yaw angle as a Rotation2d object.
     */
    private Rotation2d calculateDesiredYaw(Pose2d robotPose, Pose2d speakerPose) {
        // Calculate the relative translation from the robot to the speaker
        double deltaX = speakerPose.getX() - robotPose.getX();
        double deltaY = speakerPose.getY() - robotPose.getY();

        // Calculate the angle from the robot to the speaker
        double angleRadians = Math.atan2(deltaY, deltaX);
        Rotation2d desiredYaw = new Rotation2d(angleRadians);

        System.out.println("[ShootoSpeakerAutoAngle] Calculated desired yaw: " + desiredYaw.getDegrees() + " degrees.");
        return desiredYaw;
    }

    /**
     * Calculates the rotational rate needed to turn the robot towards the desired yaw.
     * This uses a simple proportional control for demonstration purposes.
     *
     * @param currentYaw The current yaw of the robot.
     * @param desiredYaw The desired yaw to face the speaker.
     * @return The rotational speed in radians per second.
     */
    private double calculateYawRate(Rotation2d currentYaw, Rotation2d desiredYaw) {
        double yawErrorDegrees = desiredYaw.minus(currentYaw).getDegrees();
        double kP = 0.02; // Proportional gain (tune as needed)

        // Calculate rotational speed with proportional control
        double rotationSpeed = kP * yawErrorDegrees;

        // Clamp the rotation speed to a maximum value
        double maxRotationSpeed = Math.toRadians(180); // Max 180 degrees per second
        rotationSpeed = Math.max(-maxRotationSpeed, Math.min(rotationSpeed, maxRotationSpeed));

        System.out.println("[ShootoSpeakerAutoAngle] Yaw Error: " + yawErrorDegrees + " degrees, Rotation Speed: " + Math.toDegrees(rotationSpeed) + " degrees/sec");
        return rotationSpeed;
    }

    /**
     * Calculates the target RPM for the shooter based on the distance to the speaker.
     *
     * @param distance The distance to the speaker in meters.
     * @return The calculated RPM for the shooter.
     */
    private double calculateTargetRPM(double distance) {
        // Define a formula for RPM based on distance. This could be linear or polynomial.
        // Example: a linear relationship where RPM increases as distance increases
        // Adjust coefficients as needed for your robot's requirements
        double baseRPM = Constants.ShooterConstans.SHOOTER_TARGET_RPM; // Base RPM for close range
        double rpmPerMeter = Constants.ShooterConstans.SHOOTER_RPM_Scailing_Per_Meter; // Additional RPM per meter of distance

        double targetRPM = baseRPM + (rpmPerMeter * distance);
        System.out.println("[ShootoSpeakerAutoAngle] Calculated target RPM: " + targetRPM);
        return targetRPM;
    }
}
