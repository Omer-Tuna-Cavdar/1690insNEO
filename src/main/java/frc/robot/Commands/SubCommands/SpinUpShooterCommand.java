package frc.robot.Commands.SubCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsytems.Shooter;
import frc.robot.Subsytems.Vision;

public class SpinUpShooterCommand extends Command {
    private final Shooter shooter;
    private final Vision vision;
    private final String targetName;
    private final Double fixedRPM; // If provided, use this RPM instead of calculating
    private double targetRPM;

    public SpinUpShooterCommand(Shooter shooter, Vision vision, String targetName) {
        this.shooter = shooter;
        this.vision = vision;
        this.targetName = targetName;
        this.fixedRPM = null;
        addRequirements(shooter);
    }

    public SpinUpShooterCommand(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.vision = null;
        this.targetName = null;
        this.fixedRPM = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (fixedRPM != null) {
            targetRPM = fixedRPM;
            System.out.println("[SpinUpShooterCommand] Initializing to fixed RPM: " + fixedRPM);
        } else {
            System.out.println("[SpinUpShooterCommand] Initializing using vision data for target: " + targetName);
            double distance = vision.getDistance(targetName);
            if (distance < 0.0) {
                System.out.println("[SpinUpShooterCommand] Invalid distance to target. Setting RPM to zero.");
                targetRPM = 0;
            } else {
                targetRPM = calculateTargetRPM(distance);
            }
        }
        shooter.runShooter(targetRPM);
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtTargetRPM(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        if (interrupted) {
            System.out.println("[SpinUpShooterCommand] Interrupted.");
        } else {
            System.out.println("[SpinUpShooterCommand] Completed.");
        }
    }

    private double calculateTargetRPM(double distance) {
        double baseRPM = Constants.ShooterConstans.SHOOTER_TARGET_RPM;
        double rpmPerMeter = Constants.ShooterConstans.SHOOTER_RPM_Scailing_Per_Meter;
        return baseRPM + (rpmPerMeter * distance);
    }
}
