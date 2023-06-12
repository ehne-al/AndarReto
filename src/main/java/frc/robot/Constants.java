package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

    public static class EncoderConstants{
        public static final int kEncoderTicksPerRevolution = 4096;
    }

    public static class DimensionsConstants{
        public static final double kWheelCircunference = 0.471238898;
        public static final double kTrackWidth = 0.381 * 2;

    }

    public static class DrivetrainConstants{
        public static final int leftMasterPort = 3;
        public static final int leftSlavePort = 4;
        public static final int rightMasterPort = 1;
        public static final int rightSlavePort = 2;

        public static final double kP = 2.9;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final double kF = 0.5;
        public static final int kS = 1;
        public static final int kV = 3;
        public static final int kA = 0;

        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(DimensionsConstants.kTrackWidth);

    }

    public static class AutoConstants{
        public static final int kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final int kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final int kV = 3;
        public static final int kA = 0;

    }
}
