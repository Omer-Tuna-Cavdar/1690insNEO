package frc.robot.Subsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
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

        VisionData(double distance, double tx, double ty) {
            this.distance = distance;
            this.tx = tx;
            this.ty = ty;
        }
    }

    // Cache to store VisionData for each target type
    private final Map<String, VisionData> visionDataCache = new HashMap<>();

    public Vision(String limelightName) {
        this.limelightName = limelightName;
        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue); // Get alliance color at startup
    }

    /**
     * Retrieves the Pose2d of the amp target based on AprilTag detection.
     *
     * @return Pose2d of the amp relative to the robot, or null if not detected.
     */
    public Pose2d getAmpPose() {
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        if (results == null || results.targets_Fiducials == null || results.targets_Fiducials.length == 0) {
            System.out.println("[Vision] No fiducials detected.");
            return null;
        }

        // Determine the Amp's AprilTag ID based on alliance
        int[] ampTagIDs = getAmpTagIDs();

        // Since there's only one amp per alliance, find the first matching fiducial
        LimelightTarget_Fiducial ampTarget = Arrays.stream(results.targets_Fiducials)
            .filter(target -> Arrays.stream(ampTagIDs).anyMatch(id -> id == (int) target.fiducialID))
            .findFirst()
            .orElse(null);

        if (ampTarget == null) {
            System.out.println("[Vision] Amp fiducial not detected.");
            return null;
        }

        // Retrieve the robot pose relative to the amp target as Pose2d
        Pose2d ampPose = ampTarget.getRobotPose_TargetSpace2D();

        System.out.println("[Vision] Amp pose acquired: " + ampPose);
        return ampPose;
    }

    /**
     * Determines the Amp's AprilTag IDs based on the current alliance.
     *
     * @return Array of Amp AprilTag IDs.
     */
    private int[] getAmpTagIDs() {
        return getAllianceSpecificTagIDs("Amp");
    }

    /**
     * Calculates the distance to the target based on the height difference and the vertical angle.
     *
     * @param h  The height difference between the target and the AprilTag.
     * @param ty The vertical angle to the target from the Limelight.
     * @return The calculated distance in meters.
     */
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

    /**
     * Calculates and caches the adjusted parameters for a given target type.
     *
     * @param targetType The type of target (e.g., "Speaker", "Amp", "Source").
     */
    private void calculateAndCacheParameters(String targetType) {
        double targetHeight = getTargetHeight(targetType);
        double tagHeight = getTagHeight(targetType);

        double h = targetHeight - tagHeight; // Height difference
        double[] distanceAndOffsets = getDistanceAndOffsets(targetType); // Get alliance-specific data
        double distance = distanceAndOffsets[0];
        double tx = distanceAndOffsets[1];
        double ty = distanceAndOffsets[2];

        if (distance == -1.0) {
            // Indicate invalid data by setting all values to -1.0
            visionDataCache.put(targetType, new VisionData(-1.0, -1.0, -1.0));
        } else {
            // Adjust tx and ty if needed
            double adjustedTX = calculateTXAdjustment(distance, h);
            double adjustedTY = calculateTYAdjustment(distance, h);
            visionDataCache.put(targetType, new VisionData(distance, adjustedTX, adjustedTY));
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
     * Retrieves alliance-specific distance and offsets (tx, ty) from detected AprilTags.
     */
    private double[] getDistanceAndOffsets(String targetType) {
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        if (results == null || results.targets_Fiducials == null || results.targets_Fiducials.length == 0) {
            System.out.println("[Vision] Warning: No targets detected.");
            return new double[] { -1.0, -1.0, -1.0 };
        }

        int[] tagIDs = getAllianceSpecificTagIDs(targetType);
        double totalTX = 0.0, totalTY = 0.0;
        int detectedTags = 0;

        for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
            int fiducialID = (int) target.fiducialID;

            if (Arrays.stream(tagIDs).anyMatch(id -> id == fiducialID)) {
                totalTX += target.tx;
                totalTY += target.ty;
                detectedTags++;
            }
        }

        if (detectedTags > 0) {
            double avgTX = totalTX / detectedTags;
            double avgTY = totalTY / detectedTags;

            double targetHeight = getTargetHeight(targetType);
            double tagHeight = getTagHeight(targetType);
            double h = targetHeight - tagHeight;

            double distance = calculateDistance(h, avgTY);
            return new double[] { distance, avgTX, avgTY };
        } else {
            System.out.println("[Vision] Warning: Not all tags detected for " + targetType);
            return new double[] { -1.0, -1.0, -1.0 };
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
     * Example usage: Get the amp pose and print it.
     */
    public void printAmpPose() {
        Pose2d ampPose = getAmpPose();
        if (ampPose != null) {
            System.out.println("[Vision] Amp Pose: " + ampPose);
        } else {
            System.out.println("[Vision] Amp Pose not detected.");
        }
    }
}
