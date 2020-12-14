package org.vulcanrobotics.robotcorelib.motion;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev = 103.6, gearing = 3.7, maxRPM = 1620, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "flywheelMotor", name = "Custom FLywheel Profile")
public interface FlywheelMotor {
}
