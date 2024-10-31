package frc.robot.Subsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.HelperMerhodes.LimelightHelpers;
import frc.robot.HelperMerhodes.LimelightHelpers.LimelightResults;
import frc.robot.HelperMerhodes.LimelightHelpers.LimelightTarget_Fiducial;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Vision extends SubsystemBase {
    private String limelightName;
    private final double g = 9.81; // Gravity in m/s^2
    private Alliance alliance;

    // Constants for target heights in meters
    private static final double SPEAKER_TARGET_HEIGHT = Constants.FieldConstants.SPEAKER_TARGET_HEIGHT; // Speaker target height in meters
    private static final double AMP_TARGET_HEIGHT = Constants.FieldConstants.AMP_TARGET_HEIGHT;   // Amp target height in meters
    private static final double SOURCE_TARGET_HEIGHT = Constants.FieldConstants.SOURCE_TARGET_HEIGHT; // Source target height in meters

    // Constants for AprilTag heights in meters
    private static final double SPEAKER_TAG_HEIGHT = Constants.FieldConstants.SPEAKER_TAG_HEIGHT;
    private static final double AMP_TAG_HEIGHT = Constants.FieldConstants.AMP_TAG_HEIGHT;
    private static final double SOURCE_TAG_HEIGHT = Constants.FieldConstants.SOURCE_TAG_HEIGHT;

    // Camera mounting angle in degrees (adjust based on your robot's setup)
    private static final double CAMERA_MOUNTING_ANGLE_DEGREES = 30.0; // Example value

    // Nested class to hold cached vision data
    private static class VisionData {
        double distance;
        double tx;
        double ty;
        Pose2d targetPose;

        VisionData(double distance, double tx, double ty, Pose2d targetPose) {
            this.distance = distance;
            this.tx = tx;
            this.ty = ty;
            this.targetPose = targetPose;
        }
    }

    // Cache to store VisionData for each target type
    private final Map<String, VisionData> visionDataCache = new HashMap<>();

    public Vision(String limelightName) {
        this.limelightName = limelightName;
        this.alliance = DriverStation.getAlliance().get(); // Get alliance color at startup
    }

    /**
     * Retrieves the Pose2d of the specified target based on AprilTag detection.
     *
     * @param targetType The type of target (e.g., "Amp", "Speaker", "Source").
     * @return Pose2d of the target relative to the robot, or null if not detected.
     */
    public Pose2d getTargetPose(String targetType) {
        ensureDataCached(targetType);
        VisionData data = visionDataCache.get(targetType);
        return (data != null) ? data.targetPose : null;
    }

    /**
     * Calculates and caches the adjusted parameters for a given target type.
     *
     * @param targetType The type of target (e.g., "Speaker", "Amp", "Source").
     */
    private void calculateAndCacheParameters(String targetType) {
        double targetHeight = getTargetHeight(targetType);
        double tagHeight = getTagHeight(targetType);

        double h = targetHeight - tagHeight; // Height difference
        TargetDetectionResult detectionResult = getDetectionResult(targetType); // Get detection data
        double distance = detectionResult.distance;
        double tx = detectionResult.tx;
        double ty = detectionResult.ty;
        Pose2d targetPose = detectionResult.pose;

        if (distance == -1.0) {
            // Indicate invalid data by setting all values to -1.0
            visionDataCache.put(targetType, new VisionData(-1.0, -1.0, -1.0, null));
        } else {
            // Adjust tx and ty if needed
            double adjustedTX = calculateTXAdjustment(distance, h);
            double adjustedTY = calculateTYAdjustment(distance, h);
            visionDataCache.put(targetType, new VisionData(distance, adjustedTX, adjustedTY, targetPose));
        }
    }

    /**
     * Retrieves the calculated distance to the target.
     *
     * @param targetType The type of target (e.g., "Speaker", "Amp", "Source").
     * @return The distance in meters, or -1.0 if unavailable.
     */
    public double getDistance(String targetType) {
        ensureDataCached(targetType);
        VisionData data = visionDataCache.get(targetType);
        return (data != null) ? data.distance : -1.0;
    }

    /**
     * Retrieves the calculated horizontal angle (tx) to the target.
     *
     * @param targetType The type of target (e.g., "Speaker", "Amp", "Source").
     * @return The horizontal angle in degrees, or -1.0 if unavailable.
     */
    public double getTX(String targetType) {
        ensureDataCached(targetType);
        VisionData data = visionDataCache.get(targetType);
        return (data != null) ? data.tx : -1.0;
    }

    /**
     * Retrieves the calculated vertical angle (ty) to the target.
     *
     * @param targetType The type of target (e.g., "Speaker", "Amp", "Source").
     * @return The vertical angle in degrees, or -1.0 if unavailable.
     */
    public double getTY(String targetType) {
        ensureDataCached(targetType);
        VisionData data = visionDataCache.get(targetType);
        return (data != null) ? data.ty : -1.0;
    }

    /**
     * Ensures that the vision data for the specified target type is calculated and cached.
     *
     * @param targetType The type of target (e.g., "Speaker", "Amp", "Source").
     */
    private void ensureDataCached(String targetType) {
        if (!visionDataCache.containsKey(targetType)) {
            calculateAndCacheParameters(targetType);
        }
    }

    /**
     * Determines target height based on target type.
     */
    private double getTargetHeight(String targetType) {
        switch (targetType) {
            case "Speaker":
                return SPEAKER_TARGET_HEIGHT;
            case "Amp":
                return AMP_TARGET_HEIGHT;
            case "Source":
                return SOURCE_TARGET_HEIGHT;
            default:
                throw new IllegalArgumentException("Unknown target type: " + targetType);
        }
    }

    /**
     * Determines AprilTag height based on target type.
     */
    private double getTagHeight(String targetType) {
        switch (targetType) {
            case "Speaker":
                return SPEAKER_TAG_HEIGHT;
            case "Amp":
                return AMP_TAG_HEIGHT;
            case "Source":
                return SOURCE_TAG_HEIGHT;
            default:
                throw new IllegalArgumentException("Unknown target type: " + targetType);
        }
    }

    /**
     * Retrieves detection data including distance, offsets, and pose from detected AprilTags.
     */
    private TargetDetectionResult getDetectionResult(String targetType) {
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        if (results == null || results.targets_Fiducials == null || results.targets_Fiducials.length == 0) {
            System.out.println("[Vision] Warning: No targets detected.");
            return new TargetDetectionResult(-1.0, -1.0, -1.0, null);
        }

        int[] tagIDs = getAllianceSpecificTagIDs(targetType);
        double totalTX = 0.0, totalTY = 0.0;
        int detectedTags = 0;
        Pose2d targetPose = null;

        for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
            int fiducialID = (int) target.fiducialID;

            if (Arrays.stream(tagIDs).anyMatch(id -> id == fiducialID)) {
                totalTX += target.tx;
                totalTY += target.ty;
                detectedTags++;

                // For simplicity, use the first detected target's pose
                if (targetPose == null) {
                    targetPose = target.getRobotPose_TargetSpace2D();
                }
            }
        }

        if (detectedTags > 0) {
            double avgTX = totalTX / detectedTags;
            double avgTY = totalTY / detectedTags;

            double targetHeight = getTargetHeight(targetType);
            double tagHeight = getTagHeight(targetType);
            double h = targetHeight - tagHeight;

            double distance = calculateDistance(h, avgTY);
            return new TargetDetectionResult(distance, avgTX, avgTY, targetPose);
        } else {
            System.out.println("[Vision] Warning: Not all tags detected for " + targetType);
            return new TargetDetectionResult(-1.0, -1.0, -1.0, null);
        }
    }

    /**
     * Adjusts the horizontal angle (tx) based on specific requirements.
     *
     * @param distance   The calculated distance to the target.
     * @param heightDiff The height difference between camera and target.
     * @return The adjusted tx value.
     */
    private double calculateTXAdjustment(double distance, double heightDiff) {
        // Example adjustment (modify as needed)
        double fixedOffsetDegrees = 0.0; // Replace with actual offset if any
        return fixedOffsetDegrees;
    }

    /**
     * Adjusts the vertical angle (ty) based on specific requirements.
     *
     * @param distance   The calculated distance to the target.
     * @param heightDiff The height difference between camera and target.
     * @return The adjusted ty value.
     */
    private double calculateTYAdjustment(double distance, double heightDiff) {
        // Example adjustment (modify as needed)
        double elevationOffsetDegrees = 0.0; // Replace with actual offset if any
        return elevationOffsetDegrees;
    }

    /**
     * Gets relevant tag IDs based on alliance color and target type.
     */
    private int[] getAllianceSpecificTagIDs(String targetType) {
        boolean isRedAlliance = (alliance == Alliance.Red);
        switch (targetType) {
            case "Speaker":
                return isRedAlliance ? new int[]{3, 4} : new int[]{7, 8};
            case "Amp":
                return isRedAlliance ? new int[]{5} : new int[]{6};
            case "Source":
                return isRedAlliance ? new int[]{9, 10} : new int[]{1, 2};
            default:
                throw new IllegalArgumentException("Unknown target type: " + targetType);
        }
    }

    /**
     * Clears the vision data cache periodically to ensure fresh calculations.
     */
    @Override
    public void periodic() {
        // Clear the cache at the end of each cycle to prepare for new data
        visionDataCache.clear();
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTargetCount(limelightName) > 0;
    }

    public Pose2d getLatestPose() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }

    public double getLatency() {
        return LimelightHelpers.getLatency_Pipeline(limelightName);
    }

    /**
     * Helper class to hold detection results.
     */
    private static class TargetDetectionResult {
        double distance;
        double tx;
        double ty;
        Pose2d pose;

        TargetDetectionResult(double distance, double tx, double ty, Pose2d pose) {
            this.distance = distance;
            this.tx = tx;
            this.ty = ty;
            this.pose = pose;
        }
    }
    private double calculateDistance(double h, double ty) {
        // Convert angles from degrees to radians for calculation
        double cameraAngleRadians = Math.toRadians(CAMERA_MOUNTING_ANGLE_DEGREES);
        double tyRadians = Math.toRadians(ty);

        // Total angle from the horizontal
        double totalAngle = cameraAngleRadians + tyRadians;

        // Prevent division by zero
        double tanTotalAngle = Math.tan(totalAngle);
        if (tanTotalAngle == 0) {
            System.out.println("[Vision] Warning: Total angle leads to division by zero.");
            return -1.0;
        }

        // Calculate distance using trigonometry
        double distance = h / tanTotalAngle;
        return distance;
    }

}
