package org.firstinspires.ftc.teamcode.odysseygui;

public class FieldCoordinates {

    private static final double totalMM = 3657.5;
    private static final double totalPX = 650;
    private static final double pxPerMM = totalPX/totalMM;
    private static final double mmPerPX = totalMM/totalPX;

    public static double fieldX_PX_TO_MM(double px) {
        return px * mmPerPX;
    }

    public static double fieldX_MM_TO_PX(double mm)  {
        return mm * pxPerMM;
    }

    public static double fieldY_PX_TO_MM(double px) {
        return fieldX_PX_TO_MM(totalPX-px);
    }

    public static double fieldY_MM_TO_PX(double mm) {
        return fieldX_MM_TO_PX(totalMM-mm);
    }
}
