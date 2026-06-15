package com.aaravdhawan25.pidautotuner.ftc;

import com.aaravdhawan25.pidautotuner.RelayAutoTuner;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDMaster {
    DcMotorEx motor;

    public PIDMaster(HardwareMap map){
        motor = map.get(DcMotorEx.class, TuningConfig.MOTOR_NAME);

        motor.setDirection(TuningConfig.REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        motor.setZeroPowerBehavior(TuningConfig.PositionPIDtuner
                ? DcMotorEx.ZeroPowerBehavior.BRAKE
                : DcMotorEx.ZeroPowerBehavior.FLOAT);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if (TuningConfig.PositionPIDtuner){
            int startPosition = motor.getCurrentPosition();
            double setpoint = startPosition + TuningConfig.POSITION_TARGET_TICKS;

            RelayAutoTuner tuner = new RelayAutoTuner(
                    setpoint,
                    TuningConfig.RELAY_AMPLITUDE,
                    TuningConfig.POSITION_HYSTERESIS_TICKS,
                    TuningConfig.CYCLES_TO_COLLECT,
                    TuningConfig.CYCLES_TO_IGNORE
            );
        }
    }






}
