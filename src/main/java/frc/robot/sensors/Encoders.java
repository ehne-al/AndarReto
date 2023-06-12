package frc.robot.sensors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.*;

public class Encoders{

    public double encoderTicksToMeters(double encoderticks) {
        return (encoderticks / EncoderConstants.kEncoderTicksPerRevolution) * DimensionsConstants.kWheelCircunference;
    }

    public double encoderMeterToTicks(double encoderMeters) {
        return (encoderMeters *  EncoderConstants.kEncoderTicksPerRevolution) / DimensionsConstants.kWheelCircunference;
    }

    public double getEncoderSpeedMeters(WPI_TalonSRX masterMotor){
        double metersPerSecond = masterMotor.getSelectedSensorVelocity() * 60 *(1 / EncoderConstants.kEncoderTicksPerRevolution);
        return metersPerSecond;
    }

    public double getEncoderTicks(WPI_TalonSRX masterMotor) {
        double position;
          position = -masterMotor.getSelectedSensorPosition(0);
        return position;
    }

    public double getEncoderMeters(WPI_TalonSRX masterMotor) {
        double meters = encoderTicksToMeters(getEncoderTicks(masterMotor));
        return meters;
    }

    public void resetEncoders(WPI_TalonSRX masterMotor){
        masterMotor.setSelectedSensorPosition(0);
    }

}
