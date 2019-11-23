package org.ghrobotics.frc2020.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private static final double kGearRatio = 7.29;
  private static final double kWheelRadiusInches = 3.0;

  CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANSparkMax leftSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics, getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController leftPIDController = new PIDController(2.95, 0, 0);
  PIDController rightPIDController = new PIDController(2.95, 0, 0);

  Pose2d pose = new Pose2d();

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
        rightMaster.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getSpeeds());
  }
}
