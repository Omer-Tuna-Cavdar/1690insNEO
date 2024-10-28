package frc.robot.HelperMerhodes;

public class HelperMethodes {
    public static double degreesToPosition(double degrees, double gearReduction) {
        int cpr = 2048; // Default counts per revolution
        // Calculate position in raw units
        return (degrees / 360.0) * cpr * gearReduction;
    }    
}
