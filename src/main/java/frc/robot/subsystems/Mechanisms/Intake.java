package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Relay;

public class Intake extends SubsystemBase{
    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private DigitalInput intakeSensor = new DigitalInput(7);
    private Relay LEDs = new Relay(0);

    public Intake() {
        topMotor = new TalonFX(Constants.IntakeConstants.TopMotorID);
        bottomMotor = new TalonFX(Constants.IntakeConstants.BottomMotorID);
    }

    private void setIntakeSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    public Command On() {
        return Commands.runOnce(()->{setIntakeSpeed(Constants.IntakeConstants.IntakeSpeed);}, this);
    }

    public Command Reverse() {
        return Commands.runOnce(()->{setIntakeSpeed(-Constants.IntakeConstants.IntakeSpeed);}, this);
    }

    public Command Off() {
        return Commands.runOnce(()->{setIntakeSpeed(0);}, this);
    }

    public Command Intake() {
        return Commands.either(Commands.sequence(this.On(), Commands.waitUntil(() -> !intakeSensor.get()), Commands.waitSeconds(0.05), this.Off()), Commands.none(), () -> intakeSensor.get());
    };
}
