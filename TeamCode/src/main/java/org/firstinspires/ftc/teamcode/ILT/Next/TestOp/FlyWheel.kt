package org.firstinspires.ftc.teamcode.robot.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.FlyWheel


/**
 * Flywheel Test OpMode
 *
 * Based on NextControl documentation example.
 *
 * Controls:
 *   A = Set velocity to 2000 (spin up)
 *   B = Set velocity to 0 (stop)
 *   X = Set velocity to 1000 (half speed)
 *   Y = Set velocity to 1500
 *
 *   Right Bumper = Full power (bypass PID)
 *   Left Bumper = Reverse (clear jams)
 */
@TeleOp(name = "Test - Flywheel", group = "Test")
class FlywheelTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(FlyWheel),
            BulkReadComponent
        )
    }

    override fun onInit() {
        telemetry.addLine("=== FLYWHEEL TEST ===")
        telemetry.addLine("A = 2000 vel (spin)")
        telemetry.addLine("B = 0 vel (stop)")
        telemetry.addLine("X = 1000 vel")
        telemetry.addLine("Y = 1500 vel")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // Start with flywheel stopped
        FlyWheel.controller.goal = KineticState(0.0, 0.0)
    }

    override fun onUpdate() {
        // Button presses change the target velocity
        // Using the exact pattern from NextControl docs

        if (gamepad1.a) {
            // Spin up to 2000
            FlyWheel.controller.goal = KineticState(0.0, 2000.0)
            FlyWheel.targetVelocity = 2000.0
        }

        if (gamepad1.b) {
            // Stop
            FlyWheel.controller.goal = KineticState(0.0, 0.0)
            FlyWheel.targetVelocity = 0.0
        }

        if (gamepad1.x) {
            // Half speed
            FlyWheel.controller.goal = KineticState(0.0, 1000.0)
            FlyWheel.targetVelocity = 1000.0
        }

        if (gamepad1.y) {
            // 1500
            FlyWheel.controller.goal = KineticState(0.0, 1500.0)
            FlyWheel.targetVelocity = 1500.0
        }


        if (gamepad1.left_bumper) {
            FlyWheel.fly1.power = -0.5
            FlyWheel.fly2.power = -0.5
        }

        // Telemetry
        telemetry.addLine("=== FLYWHEEL TEST ===")
        telemetry.addLine()
        telemetry.addData("Target Vel", "%.0f".format(FlyWheel.targetVelocity))
        telemetry.addData("Current Vel", "%.0f".format(FlyWheel.getCurrentVelocity()))
        telemetry.addData("RPM", "%.0f".format(FlyWheel.getRPM()))
        telemetry.addData("Power", "%.2f".format(FlyWheel.fly1.power))
        telemetry.addData("At Speed", FlyWheel.isAtTargetVelocity())
        telemetry.addLine()
        telemetry.addLine("A=2000  B=Stop  X=1000  Y=1500")
        telemetry.addLine("RB=FullPower  LB=Reverse")
    }
}