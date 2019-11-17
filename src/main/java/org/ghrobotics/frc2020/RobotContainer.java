package org.ghrobotics.frc2020;

import java.util.List;

import org.ghrobotics.frc2020.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import static org.ghrobotics.frc2020.Constants.DriveConstants.kAVoltSecondsSquaredPerMeter;
import static org.ghrobotics.frc2020.Constants.DriveConstants.kMaxAccelerationFPS;
import static org.ghrobotics.frc2020.Constants.DriveConstants.kMaxVelocityFPS;
import static org.ghrobotics.frc2020.Constants.DriveConstants.kP;
import static org.ghrobotics.frc2020.Constants.DriveConstants.kSVolts;
import static org.ghrobotics.frc2020.Constants.DriveConstants.kVVoltSecondsPerMeter;

public class RobotContainer {

  private final Drivetrain m_drive = new Drivetrain();


  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Units.feetToMeters(kMaxVelocityFPS), Units.feetToMeters(kMaxAccelerationFPS))
            .setKinematics(m_drive.getKinematics());

    // An example trajectory to follow
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d()),
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        new Pose2d(3, 0, new Rotation2d()),
        config
    );

    RamseteCommand command = new RamseteCommand(
        trajectory,
        m_drive::getPose,
        m_drive.getController(),
        kSVolts, kVVoltSecondsPerMeter, kAVoltSecondsSquaredPerMeter,
        m_drive.getKinematics(),
        m_drive::getLeftSpeedMetersPerSecond,
        m_drive::getRightSpeedMetersPerSecond,
        new PIDController(kP, 0, 0),
        new PIDController(kP, 0, 0),
        m_drive::tankDriveVolts,
        m_drive
    );

    return command.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
