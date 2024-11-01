package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    
    // Use CommandPS4Controller for PS5 controller support with command-based methods
    final CommandPS5Controller driverController = new CommandPS5Controller(0);

    // The robot's subsystems and commands are defined here...

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(Constants.Subsytems.SWERVE_SUBSYSTEM,
        () -> -MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        driverController.triangle()::getAsBoolean,
        driverController.cross()::getAsBoolean,
        driverController.square()::getAsBoolean,
        driverController.circle()::getAsBoolean);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = Constants.Subsytems.SWERVE_SUBSYSTEM.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX(),
        () -> driverController.getRightY());
    
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAngularVelocity = Constants.Subsytems.SWERVE_SUBSYSTEM.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX() * -1);

        Command driveFieldOrientedDirectAngleSim = Constants.Subsytems.SWERVE_SUBSYSTEM.simDriveCommand(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));
    Command ShootoSpeakerAutoAngle  = new ShootoSpeakerAutoAngleCommandGroup(Constants.Subsytems.pivot, Constants.Subsytems.vision, Constants.Subsytems.shooter, Constants.Subsytems.intake, Constants.Subsytems.SWERVE_SUBSYSTEM);
    Command DriveToAmpAndShootCommand = new DriveToAmpAndShootCommandGroup(Constants.Subsytems.SWERVE_SUBSYSTEM, Constants.Subsytems.vision, Constants.Subsytems.shooter, Constants.Subsytems.pivot, Constants.Subsytems.intake);
    Command IntakeNote = new IntakeNote(Constants.Subsytems.intake);
    private final Notifier intakeNotifier = new Notifier(() -> IntakeNote.schedule());
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        intakeNotifier.startPeriodic(0.0005);  // 0.5 ms interval same as the beam break sensor
        configureBindings();    
        NamedCommands.registerCommand("ShootNote", ShootoSpeakerAutoAngle);
        NamedCommands.registerCommand("IntakeNote", IntakeNote);

    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings()
    {
        // Configure the button bindings
            driverController.cross().onTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::zeroGyro));
            driverController.L1().whileTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::lock, Constants.Subsytems.SWERVE_SUBSYSTEM).repeatedly());
            driverController.R2().onTrue(ShootoSpeakerAutoAngle);
            driverController.L2().onTrue(DriveToAmpAndShootCommand);
            driverController.R1().onTrue(IntakeNote);
            Constants.Subsytems.SWERVE_SUBSYSTEM.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return Constants.Subsytems.SWERVE_SUBSYSTEM.getAutonomousCommand("AutoDeneme");
    }
    public Pose2d getAutoStartPose(){
        return Constants.Subsytems.SWERVE_SUBSYSTEM.getautoStartPose("AutoDeneme");
    }

    public void setDriveMode()
    {
        configureBindings();
    }

    public void setMotorBrake(boolean brake)
    {
        Constants.Subsytems.SWERVE_SUBSYSTEM.setMotorBrake(brake);
    }


}
