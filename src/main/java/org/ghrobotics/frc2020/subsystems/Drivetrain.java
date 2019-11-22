package org.ghrobotics.frc2020.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.ghrobotics.frc2020.Constants.DriveConstants.kRotationsToMetersFactor;
import static org.ghrobotics.frc2020.Constants.DriveConstants.kTrackWidthInches;

public class Drivetrain extends SubsystemBase {

  // The current robot pose
  Pose2d m_pose = new Pose2d();

  // Create motors
  private CANSparkMax m_leftMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_rightMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_leftSlave1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax m_rightSlave1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

  // Gyro
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Kinematics and odometry
  private DifferentialDriveKinematics m_kinematics
      = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackWidthInches));

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_kinematics, getHeading());
  private RamseteController m_controller = new RamseteController(2.0, 0.7);

  // Ctor
  public Drivetrain() {
    m_leftSlave1.follow(m_leftMaster);
    m_rightSlave1.follow(m_rightMaster);

    m_leftMaster.setInverted(false);
    m_rightMaster.setInverted(true);
  }

  // Drive the robot with voltage inputs
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.set(leftVolts / 12.0);
    m_rightMaster.set(rightVolts / 12.0);
  }

  // Get the current speeds
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftMaster.getEncoder().getVelocity() * kRotationsToMetersFactor / 60,
        m_rightMaster.getEncoder().getVelocity() * kRotationsToMetersFactor / 60
    );
  }
  // Get the current heading
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  // Getter for kinematics
  public final DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public RamseteController getController() {
    return m_controller;
  }

  // Get the pose
  public Pose2d getPose() {
    return m_pose;
  }

  @Override
  public void periodic() {
    m_pose = m_odometry.update(getHeading(), getSpeeds());
  }
}
