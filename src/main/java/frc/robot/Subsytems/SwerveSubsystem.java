// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    /**
     * AprilTag field layout.
     */
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    /**
     * Vision system instance.
     */
    private final Vision vision = Constants.Subsytems.vision;

    /**
     * PID Controller for rotation control using Limelight.
     */
    private final PIDController rotationController = new PIDController(0.05, 0.0, 0.0);

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
        // In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
        // The gear ratio is 6.75 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
        System.out.println("\"conversionFactors\": {");
        System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
        System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
        System.out.println("}");

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true,
                true,
                0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false,
                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible

        // Initialize the Vision system
       

        setupPathPlanner();
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED);
    }

    @Override
    public void periodic() {
        // Update odometry
        swerveDrive.updateOdometry();

        // Get vision data
        if (vision != null && vision.hasTarget()) {
            Pose2d visionPose = vision.getLatestPose();
            if (visionPose != null) {
                double latency = vision.getLatency();
                double timeStamp = Timer.getFPGATimestamp() - (latency / 1000.0);
                swerveDrive.addVisionMeasurement(visionPose, timeStamp);
            }
        } else if (vision == null) {
            System.out.println("[SwerveSubsystem] Vision subsystem is not initialized.");
        }
        SwerveModule[] modules = swerveDrive.getModules();
        for (int i = 0; i < modules.length; i++) {
            SwerveModuleState state = modules[i].getState();
            Logger.recordOutput("SwerveSubsystem/Module" + i + "/Angle", state.angle.getDegrees());
            Logger.recordOutput("SwerveSubsystem/Module" + i + "/Speed", state.speedMetersPerSecond);
        }
    
        // Log robot pose
        Rotation2d pitch = swerveDrive.getPitch(); // Rotation around the X-axis
Rotation2d roll = swerveDrive.getRoll();   // Rotation around the Y-axis
Rotation2d yaw = getHeading();             // Rotation around the Z-axis (already obtained)

double x = getPose().getX();
double y = getPose().getY();
double z = 0 ;
        Pose2d pose = getPose();
        Logger.recordOutput("SwerveSubsystem/Pose/X", pose.getX());
        Logger.recordOutput("SwerveSubsystem/Pose/Y", pose.getY());
        Logger.recordOutput("SwerveSubsystem/Pose/Rotation", pose.getRotation().getDegrees());
        Logger.recordOutput("SwerveSubsystem/Pose2d", pose);
        Pose3d pose3d = new Pose3d(
    new Translation3d(x,y,z),
    new Rotation3d(
        pitch.getRadians(),
        roll.getRadians(),
        yaw.getRadians()
    )
);
    Logger.recordOutput("SwerveSubsystem/Pose3d", pose3d);
    Logger.recordOutput("SwerveSubsystem/Odometry", getPose());

    }
    

    

    @Override
    public void simulationPeriodic() {
        // Simulation-specific periodic code, if needed.
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        AutonConstants.TRANSLATION_PID,
                        // Translation PID constants
                        AutonConstants.ANGLE_PID,
                        // Rotation PID constants
                        4.5,
                        // Max module speed, in m/s
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                        // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance().get();
                    return alliance == DriverStation.Alliance.Red;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Get the distance to the speaker.
     *
     * @return Distance to speaker in meters.
     */
    public double getDistanceToSpeaker() {
        int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
        Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).orElse(null);
        if (speakerAprilTagPose == null) {
            return 0.0;
        }
        return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
    }

    /**
     * Get the yaw to aim at the speaker.
     *
     * @return {@link Rotation2d} of which you need to achieve.
     */
    public Rotation2d getSpeakerYaw() {
        int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
        Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).orElse(null);
        if (speakerAprilTagPose == null) {
            return Rotation2d.fromDegrees(0);
        }
        Pose2d robotPose = getPose();

        // Calculate the angle from the robot to the speaker AprilTag
        Translation2d relativePosition = speakerAprilTagPose.toPose2d().getTranslation().minus(robotPose.getTranslation());

        double angleToSpeaker = Math.atan2(relativePosition.getY(), relativePosition.getX());

        Rotation2d angle = new Rotation2d(angleToSpeaker);

        // The desired heading is the angle to the speaker
        return angle;
    }

    /**
     * Aim the robot at the speaker.
     *
     * @param tolerance Tolerance in degrees.
     * @return Command to turn the robot to the speaker.
     */
    public Command aimAtSpeaker(double tolerance) {
        SwerveController controller = swerveDrive.getSwerveController();
        return Commands.run(
                () -> {
                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,
                            0,
                            controller.headingCalculate(getHeading().getRadians(),
                                    getSpeakerYaw().getRadians()),
                            getHeading()));
                }).until(() -> Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) < tolerance);
    }

    /**
     * Aim the robot at the target returned by the Vision system.
     *
     * @return A {@link Command} which will run the alignment.
     */
    public Command aimAtTarget() {
        return Commands.run(() -> {
            if (vision.hasTarget()) {
                double tx = vision.getTX(null);
                double rotationSpeed = rotationController.calculate(tx, 0);
                drive(new ChassisSpeeds(0, 0, rotationSpeed));
            } else {
                // Optionally, stop the robot if no target is detected
                drive(new ChassisSpeeds(0, 0, 0));
            }
        }, this);
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return PathPlannerAuto command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto(pathName);
    }
    public Pose2d getautoStartPose(String pathName){
        return PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
    }
    
    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumVelocity(), 4.0,
                swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Enable if needed
        return Commands.run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        }, this);
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return Commands.run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                    true,
                    false);
        }, this);
    }

    /**
     * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is towards the front and positive y is towards the left.
     *                      In field-relative mode, positive x is away from the alliance wall (field North) and positive y is
     *                      towards the left wall when looking through the driver station glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field-oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot-oriented velocity.
     *
     * @param velocity Robot-oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red.
     *
     * @return true if the red alliance, false if blue.
     */
    private boolean isRedAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward.
     * If red alliance, rotate the robot 180 after the drivebase zero command.
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading; this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
     * 90 degrees.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot.
     *
     * @return A ChassisSpeeds object of the current field-relative velocity.
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot.
     *
     * @return A {@link ChassisSpeeds} object of the current velocity.
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} for the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The pitch as a {@link Rotation2d} angle.
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        return Commands.run(() -> {
            double xSpeed = Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity();
            double ySpeed = Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity();
            double rotSpeed = Math.pow(rotation.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity();
    
            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
            swerveDrive.drive(speeds);
        }, this);
    }
    

    public Command sysIdDriveMotorCommand() {
        return Commands.run(() -> {
            // Implement motor testing logic here
            swerveDrive.drive(new Translation2d(0, 0), 0, false, true);
        }, this);
    }

    public Command driveToDistanceCommand(double distanceMeters, double speedMetersPerSecond) {
        return Commands.run(() -> {
            // Implement driving logic here
            // You may need to use encoders to track distance
        }, this);
    }

 public void centerModules() {
        SwerveModuleState[] centeredStates = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        };
        swerveDrive.setModuleStates(centeredStates, false);
    }

    public Command centerModulesCommand() {
        return Commands.runOnce(() -> {
            centerModules();
        }, this);
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }
    public SwerveModulePosition[] getModulePositions() {
        SwerveModule[] modules = swerveDrive.getModules();
        SwerveModulePosition [] modulePositions = new SwerveModulePosition [modules.length];
        
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }
    

}
