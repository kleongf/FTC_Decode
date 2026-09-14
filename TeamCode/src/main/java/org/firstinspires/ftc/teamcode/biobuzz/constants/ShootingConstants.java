package org.firstinspires.ftc.teamcode.biobuzz.constants;

import org.firstinspires.ftc.teamcode.lib.math.LookupTable;

import java.util.function.Function;

public class ShootingConstants {
    public static LookupTable pollenSpeedTable = new LookupTable();
    public static LookupTable pollenHoodTable = new LookupTable();
    public static LookupTable nectarSpeedTable = new LookupTable();
    public static LookupTable nectarHoodTable = new LookupTable();
    public static int MAX_TOF_ITERATIONS = 10;
    public static final Function<Double, Double> POLLEN_TOF_FUNCTION = x -> 0.0031 * x + 0.46; // function of distance
    public static final Function<Double, Double> NECTAR_TOF_FUNCTION = x -> 0.0031 * x + 0.46; // function of distance
    private static void addPollenData(double distance, double hoodAngle, double wheelSpeed) {
        pollenSpeedTable.addData(distance, wheelSpeed);
        pollenHoodTable.addData(distance, hoodAngle);
    }

    private static void addNectarData(double distance, double hoodAngle, double wheelSpeed) {
        nectarSpeedTable.addData(distance, wheelSpeed);
        nectarHoodTable.addData(distance, hoodAngle);
    }

    static {
        addNectarData(30, 0.1, 1000);
        addPollenData(30, 0.1, 800);
    }
}
