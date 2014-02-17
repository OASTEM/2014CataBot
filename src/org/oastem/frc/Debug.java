package org.oastem.frc;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;

/**
 * A debug class.
 *
 * @author KTOmega
 */
public class Debug {

    private static Line[] lines = new Line[]{
        Line.kMain6,
        Line.kUser2,
        Line.kUser3,
        Line.kUser4,
        Line.kUser5,
        Line.kUser6
    };
    private static DriverStationLCD ds = DriverStationLCD.getInstance();

    /**
     * Log to Driver Station LCD.
     *
     * @param ln The line number from 1 to 6.
     * @param col The column - either 1 or 2.
     * @param text The line to send.
     */
    public static void log(int ln, int col, String text) {
        Line line = lines[ln - 1];
        int pos = col == 1 ? 1 : (DriverStationLCD.kLineLength / 2);

        text = text.trim();//.substring(0, (DriverStationLCD.kLineLength / 2) - 1); // Constrain

        clearLine(ln);
        ds.println(line, pos, text);
        update();
    }

    public static void log(String[] text) {
        for (int i = 0; i < text.length; i++) {
            if (text[i] == null || text[i].trim().equals("")) {
                continue;
            }
            Line line = lines[i % 6];
            int pos = 1;
            String out = text[i].trim();
            /*if (text.length > 6) {
                pos = i < 6 ? 1 : DriverStationLCD.kLineLength / 2;
                out = out.substring(0, (DriverStationLCD.kLineLength / 2) - 1); // Constrain
            }*/

            ds.println(line, pos, out);
        }

        update();
    }
    
    public static void clearLine(int line) {
        String pad = "";

        for (int i = 0; i < DriverStationLCD.kLineLength; i++) {
            pad += " ";
        }

        ds.println(lines[line - 1], 1, pad);

        update();
    }

    public static void clear() {
        String pad = "";

        for (int i = 0; i < DriverStationLCD.kLineLength; i++) {
            pad += " ";
        }

        for (int i = 0; i < 6; i++) {
            ds.println(lines[i], 1, pad);
        }

        update();
    }

    private static void update() {
        ds.updateLCD();
    }
}