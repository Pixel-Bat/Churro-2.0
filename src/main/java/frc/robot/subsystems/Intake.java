package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorCANID.IntakeID.intakeMotorCANID);

    public Intake() {

    }
}
