package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Motors.Mortar.Hardstop;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

public class Mortar extends SubsystemBase {
    // private boolean allowMovement = true;

    private SparkMax seath;
    private SparkMax flywheel;

    private SparkMaxConfig seathConfig;
    private SparkMaxConfig flywheelConfig;

    private SparkAbsoluteEncoder seathEncoder;
    private SparkClosedLoopController seathPID;
    private double lastPIDPosition = 0.0;

    // private SparkAbsoluteEncoder flywheelEncoder;

    public Mortar() {
        seath = new SparkMax(Constants.Motors.Mortar.kSeathID, MotorType.kBrushless);
        seathConfig = new SparkMaxConfig();

        seathConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake).closedLoop
                .p(Constants.Motors.Mortar.PIDValues.kSeathP)
                .i(Constants.Motors.Mortar.PIDValues.kSeathI)
                .d(Constants.Motors.Mortar.PIDValues.kSeathD);
        seathConfig.closedLoop.outputRange(-1d, 1d);
        seath.configure(seathConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        seathEncoder = seath.getAbsoluteEncoder();
        seathPID = seath.getClosedLoopController();

        flywheel = new SparkMax(Constants.Motors.Mortar.kFlywheelID, MotorType.kBrushless);
        flywheelConfig = new SparkMaxConfig();
        flywheelConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // flywheelEncoder = flywheel.getAbsoluteEncoder();

        lastPIDPosition = seathEncoder.getPosition();
        seathPID.setReference(lastPIDPosition, ControlType.kPosition);
    }

    public boolean C_SeathFront() {
        return seathEncoder.getPosition() >= Constants.Motors.Mortar.Hardstop.kSeathFront;
    }

    public boolean C_SeathBack() {
        return seathEncoder.getPosition() <= Constants.Motors.Mortar.Hardstop.kSeathBack;
    }

    public void setSeathAngle(double d) {
        lastPIDPosition = MathUtil.clamp(
                d,
                Constants.Motors.Mortar.Hardstop.kSeathBack,
                Constants.Motors.Mortar.Hardstop.kSeathFront);
        seathPID.setReference(lastPIDPosition, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        double pos = seathEncoder.getPosition();

        boolean overFront = pos >= Hardstop.kSeathFront;
        boolean underBack = pos <= Hardstop.kSeathBack;

        // Direction-aware hardstop
        boolean wantsForward = lastPIDPosition > pos + 1e-4;
        boolean wantsBack = lastPIDPosition < pos - 1e-4;

        if ((overFront && wantsForward) || (underBack && wantsBack)) {
            // Hard override: stop output *and* hold current position as the new setpoint
            seath.set(0);
            lastPIDPosition = pos;
            seathPID.setReference(lastPIDPosition, ControlType.kPosition);
        }
    }
}
