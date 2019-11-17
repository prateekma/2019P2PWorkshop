package org.ghrobotics.frc2020;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {
  public static class DriveConstants {
    public static final int kLeftMasterId = 1;
    public static final int kLeftSlave1Id = 2;
    public static final int kRightMasterId = 3;
    public static final int kRightSlave1Id = 4;

    public static final double kTrackWidthInches = 29.0;
    public static final double kGearRatio = 7.29;
    public static final double kWheelRadiusInches = 3.0;

    public static final double kSVolts = 0.2;
    public static final double kVVoltSecondsPerMeter = 1.96;
    public static final double kAVoltSecondsSquaredPerMeter = 0.06;

    public static final double kP = 0.1;

    public static double kRotationsToMetersFactor = 1 / kGearRatio
        * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches);

    public static final double kMaxVelocityFPS = 7.0;
    public static final double kMaxAccelerationFPS = 7.0;

  }
}
