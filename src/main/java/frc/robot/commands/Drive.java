package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase{
    private Drivetrain drivetrain;
    private XboxController controller = new XboxController(0);

    public Drive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double targetVelocity_UnitsPer100ms = controller.getLeftY() * 500.0 * 4096 / 600;

        drivetrain.setVelocity(drivetrain.getLeftMaster(), targetVelocity_UnitsPer100ms);
        drivetrain.setVelocity(drivetrain.getRightMaster(), targetVelocity_UnitsPer100ms);

        SmartDashboard.putNumber("error left", drivetrain.getLeftMaster().getClosedLoopError(0));
        SmartDashboard.putNumber("error right", drivetrain.getRightMaster().getClosedLoopError(0));
        SmartDashboard.putNumber("vel left", (600 * drivetrain.getVelocity(drivetrain.getLeftMaster())) / 4096);
        SmartDashboard.putNumber("vel right", (600 * drivetrain.getVelocity(drivetrain.getRightMaster())) / 4096);
        SmartDashboard.putNumber("target vel", (600 * targetVelocity_UnitsPer100ms) / 4096);


    }

}
