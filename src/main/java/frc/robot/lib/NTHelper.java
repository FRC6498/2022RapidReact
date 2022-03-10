// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class NTHelper {
    private static NetworkTable table;

    public NTHelper() {
      table = NetworkTableInstance.getDefault().getTable("team6498");
    }

    public static double getDouble(String entry) {
        return table.getEntry(entry).getDouble(0.0);
    }

    public static void setDouble(String entry, double value) {
        table.getEntry(entry).setDouble(value);
    }

    public static void setString(String entry, String value) {
        table.getEntry(entry).setString(value);
    }
}
