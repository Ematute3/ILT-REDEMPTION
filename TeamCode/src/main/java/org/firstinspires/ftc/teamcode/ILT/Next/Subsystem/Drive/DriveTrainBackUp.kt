package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive
/*
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs


@Configurable
object DriveTrainBackUp: Subsystem {
    val fL = MotorEx("fl").reversed().brakeMode()
    val fR = MotorEx("fr").brakeMode()
    val bL = MotorEx("bl").reversed().brakeMode()
    val bR = MotorEx("br").brakeMode()
    val imu = IMUEx("imu", Direction.RIGHT, Direction.UP)




    @JvmField
    var sensistivity = 1.0


    override val defaultCommand: Command
        get() = MecanumDriverControlled(
            fL,
            fR,
            bL,
            bR,
            -Gamepads.gamepad1.leftStickY.map { it * sensistivity },
            Gamepads.gamepad1.leftStickX.map { it * sensistivity },
            Gamepads.gamepad1.rightStickX.map { it * sensistivity },
            FieldCentric(imu)
        )
    private fun updateZeroPowerBehavior() {
        // Small epsilon so noisy values like 0.0001 don't break the logic
        val eps = 1e-3

        val stopped =
            abs(fL.power) < eps &&
                    abs(fR.power) < eps &&
                    abs(bL.power) < eps &&
                    abs(bR.power) < eps

        val mode = if (stopped) DcMotor.ZeroPowerBehavior.BRAKE else DcMotor.ZeroPowerBehavior.FLOAT

        fL.zeroPowerBehavior = mode
        fR.zeroPowerBehavior = mode
        bL.zeroPowerBehavior = mode
        bR.zeroPowerBehavior = mode
    }

    override fun periodic() {
        // Called every loop by NextFTC → keep brake/float in sync with motion
        updateZeroPowerBehavior()
    }

    fun ResetImu(){
        imu.zeroed()
    }
}


 */
