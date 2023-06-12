// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.DriveCTRE;
import frc.robot.commands.DriveWPI;
import frc.robot.subsystems.Drivetrain;
import java.util.List;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final Drivetrain drivetrain = new Drivetrain();

    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    XboxController controller = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();

    // drivetrain.setDefaultCommand(
    //     new RunCommand( () -> drivetrain.drive
    //         (speedLimiter.calculate(controller.getLeftY())*3,
    //         rotLimiter.calculate(controller.getRightX())*3), drivetrain
    //     )
    // );
    drivetrain.setDefaultCommand(new DriveCTRE(drivetrain));
    //drivetrain.setDefaultCommand(new DriveWPI(drivetrain));

  }

  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(controller, Button.kA.value)
        .onTrue(new InstantCommand(() -> drivetrain.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> drivetrain.setMaxOutput(0.7)));
  }


  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DrivetrainConstants.kS,
                DrivetrainConstants.kV,
                DrivetrainConstants.kA),
            DrivetrainConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            // Pass config
            config);

    
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DrivetrainConstants.kS,
                DrivetrainConstants.kV,
                DrivetrainConstants.kA),
            DrivetrainConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(DrivetrainConstants.kP, 0, 0),
            new PIDController(DrivetrainConstants.kP, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    drivetrain.printEncoders();

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
