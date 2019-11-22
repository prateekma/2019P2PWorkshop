package org.ghrobotics.frc2020;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {
  public static class DriveConstants {
    public static final double kTrackWidthInches = 29.0;
    public static final double kGearRatio = 7.29;
    public static final double kWheelRadiusInches = 3.0;

    public static final double kSVolts = 0.2;
    public static final double kVVoltSecondsPerMeter = 1.96;
    public static final double kAVoltSecondsSquaredPerMeter = 0.06;

    public static final double kP = 0.236 * 12;
    public static final double kMaxVelocityFPS = 0.2;
    public static final double kMaxAccelerationFPS = 0.2;
    public static double kRotationsToMetersFactor = 1 / kGearRatio
        * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches);

  }
}
