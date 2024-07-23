package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swerve.Swerve;

public class XAim extends Command{

    PIDController xController;
    Swerve swerve;

    public XAim(Swerve swerve) {
        this.xController = new PIDController(2, 0, 0);
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double angle = LimelightHelpers.getTX("limelight-bulah");
        double output = xController.calculate(angle, 0);
        swerve.drive(new Translation2d(), Math.toRadians((angle == 0)? 20 : output), true, true);
    }

}
