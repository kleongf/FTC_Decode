package org.firstinspires.ftc.teamcode.biobuzz.constants;

import org.firstinspires.ftc.teamcode.lib.math.LookupTable;

public class ShootingConstants {
    public static LookupTable pollenSpeedTable = new LookupTable();
    public static LookupTable pollenHoodTable = new LookupTable();
    public static LookupTable nectarSpeedTable = new LookupTable();
    public static LookupTable nectarHoodTable = new LookupTable();
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
