// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.sensors.Encoders;
import frc.robot.sensors.Pigeon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
  public static final double kMaxSpeed = 1.0;
  public static final double kMaxAngularSpeed = 2 * Math.PI; 

  private Encoders encoder = new Encoders();

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DrivetrainConstants.leftMasterPort);
  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(DrivetrainConstants.leftSlavePort);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DrivetrainConstants.rightMasterPort);
  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(DrivetrainConstants.rightSlavePort);

  private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave);
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightSlave);

  private final Pigeon pigeon = new Pigeon();

  private final PIDController leftPIDController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private final PIDController rightPIDController = new PIDController(DrivetrainConstants.kP + 1, DrivetrainConstants.kI, DrivetrainConstants.kD);

  private final DifferentialDriveOdometry odometry;

  private final DifferentialDrive diffdrive = new DifferentialDrive(leftGroup, rightGroup);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV);

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.configNominalOutputForward(0, 20);
		leftMaster.configNominalOutputReverse(0, 20);
		leftMaster.configPeakOutputForward(1, 20);
		leftMaster.configPeakOutputReverse(-1, 20);

    rightMaster.configNominalOutputForward(0, 20);
		rightMaster.configNominalOutputReverse(0, 20);
		rightMaster.configPeakOutputForward(1, 20);
		rightMaster.configPeakOutputReverse(-1, 20);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);

    rightMaster.setSensorPhase(true);

    rightGroup.setInverted(true);
    leftGroup.setInverted(false);

    leftMaster.config_kF(0, DrivetrainConstants.kF, 20);
		leftMaster.config_kP(0, DrivetrainConstants.kP, 20);
		leftMaster.config_kI(0, DrivetrainConstants.kI, 20);
		leftMaster.config_kD(0, DrivetrainConstants.kD, 20);

    rightMaster.config_kF(0, DrivetrainConstants.kF, 20);
		rightMaster.config_kP(0, DrivetrainConstants.kP, 20);
		rightMaster.config_kI(0, DrivetrainConstants.kI, 20);
		rightMaster.config_kD(0, DrivetrainConstants.kD, 20);
  
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(pigeon.getHeading()), leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public WPI_TalonSRX getLeftMaster(){
    return leftMaster;
  }

  public void setVelocity(WPI_TalonSRX master, double vel){
    master.set(ControlMode.Velocity, vel);
  }

  public double getVelocity(WPI_TalonSRX master){
    return master.getSelectedSensorVelocity(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoder.getEncoderSpeedMeters(leftMaster), encoder.getEncoderSpeedMeters(rightMaster));
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        leftPIDController.calculate(encoder.getEncoderSpeedMeters(leftMaster), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(encoder.getEncoderSpeedMeters(rightMaster), speeds.rightMetersPerSecond);
    leftGroup.setVoltage(leftOutput + leftFeedforward);
    rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    if (Math.abs(xSpeed) < 0.1){
      xSpeed = 0;
    }
    if (Math.abs(rot) < 0.1){
      rot = 0;
    }
    var wheelSpeeds = DrivetrainConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(-xSpeed, 0.0, -rot));
    setSpeeds(wheelSpeeds);

  }

  public void setMaxOutput(double maxOutput) {
    diffdrive.setMaxOutput(maxOutput);
  }

  public void printEncoders(){
    System.out.println(encoder.getEncoderTicks(leftMaster));
    System.out.println(encoder.getEncoderTicks(rightMaster));
  }

  public WPI_TalonSRX getRightMaster(){
    return rightMaster;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    diffdrive.feed();
  }

  public void updateOdometry() {
    odometry.update(
      Rotation2d.fromDegrees(pigeon.getHeading()), encoder.getEncoderMeters(leftMaster), encoder.getEncoderMeters(rightMaster));
  }

  public void resetOdometry(Pose2d pose) {
    //encoder.resetEncoders(leftMaster);
    //encoder.resetEncoders(rightMaster);
    pigeon.reset();
    odometry.resetPosition(
      Rotation2d.fromDegrees(pigeon.getHeading()), encoder.getEncoderMeters(leftMaster), encoder.getEncoderMeters(rightMaster), pose);
  }
}
