package org.firstinspires.ftc.teamcode.robot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.FlyWheel

import kotlin.math.abs

@TeleOp(name = "Flywheel PID Tuner", group = "Testing")
class FlywheelTuner : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(FlyWheel),
            BulkReadComponent,
            BindingsComponent
        )
    }

    // PID coefficients - start with current config values
    private var kP = RobotConfig.FlywheelConfig.pid.kP
    private var kI = RobotConfig.FlywheelConfig.pid.kI
    private var kD = RobotConfig.FlywheelConfig.pid.kD

    // Feedforward coefficients
    private var kV = RobotConfig.FlywheelConfig.feedforward.kV
    private var kS = RobotConfig.FlywheelConfig.feedforward.kS
    private var kA = RobotConfig.FlywheelConfig.feedforward.kA

    // Target velocity settings
    private var targetVelocity = 500.0
    private val velocityStep = 50.0
    private val velocityPresets = listOf(0.0, 300.0, 500.0, 700.0, 1000.0, 1500.0)
    private var currentPresetIndex = 2

    // Tuning mode
    private enum class TuningMode { PID, FEEDFORWARD, VELOCITY }
    private var currentMode = TuningMode.VELOCITY

    // Adjustment steps
    private var pidStep = 0.01
    private var ffStep = 0.01

    override fun onInit() {
        telemetry.addLine("Flywheel PID Tuner Ready")
        telemetry.addLine("Use gamepad controls to tune")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        bindControls()
        FlyWheel.setTargetVelocity(targetVelocity)
    }

    private fun bindControls() {
        // ==================== MODE SELECTION ====================
        // D-Pad Up/Down to switch modes
        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            currentMode = when (currentMode) {
                TuningMode.VELOCITY -> TuningMode.PID
                TuningMode.PID -> TuningMode.FEEDFORWARD
                TuningMode.FEEDFORWARD -> TuningMode.VELOCITY
            }
        }

        Gamepads.gamepad1.dpadDown whenBecomesTrue {
            currentMode = when (currentMode) {
                TuningMode.VELOCITY -> TuningMode.FEEDFORWARD
                TuningMode.FEEDFORWARD -> TuningMode.PID
                TuningMode.PID -> TuningMode.VELOCITY
            }
        }

        // ==================== VELOCITY CONTROLS ====================
        // Left/Right Bumper for velocity presets
        Gamepads.gamepad1.leftBumper whenBecomesTrue {
            if (currentPresetIndex > 0) {
                currentPresetIndex--
                targetVelocity = velocityPresets[currentPresetIndex]
                FlyWheel.setTargetVelocity(targetVelocity)
            }
        }

        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            if (currentPresetIndex < velocityPresets.size - 1) {
                currentPresetIndex++
                targetVelocity = velocityPresets[currentPresetIndex]
                FlyWheel.setTargetVelocity(targetVelocity)
            }
        }

        // Left stick Y for fine velocity adjustment
        // (handled in onUpdate for continuous control)

        // ==================== PID TUNING ====================
        // Triangle/Y - Increase P
        Gamepads.gamepad1.triangle whenBecomesTrue {
            if (currentMode == TuningMode.PID) {
                kP += pidStep
                updateController()
            }
        }

        // Cross/A - Decrease P
        Gamepads.gamepad1.cross whenBecomesTrue {
            if (currentMode == TuningMode.PID) {
                kP = (kP - pidStep).coerceAtLeast(0.0)
                updateController()
            }
        }

        // Circle/B - Increase D
        Gamepads.gamepad1.circle whenBecomesTrue {
            if (currentMode == TuningMode.PID) {
                kD += pidStep * 0.1
                updateController()
            }
        }

        // Square/X - Decrease D
        Gamepads.gamepad1.square whenBecomesTrue {
            if (currentMode == TuningMode.PID) {
                kD = (kD - pidStep * 0.1).coerceAtLeast(0.0)
                updateController()
            }
        }

        // ==================== FEEDFORWARD TUNING ====================
        // Same buttons, but for FF when in FF mode
        Gamepads.gamepad1.triangle whenBecomesTrue {
            if (currentMode == TuningMode.FEEDFORWARD) {
                kV += ffStep
                updateController()
            }
        }

        Gamepads.gamepad1.cross whenBecomesTrue {
            if (currentMode == TuningMode.FEEDFORWARD) {
                kV = (kV - ffStep).coerceAtLeast(0.0)
                updateController()
            }
        }

        Gamepads.gamepad1.circle whenBecomesTrue {
            if (currentMode == TuningMode.FEEDFORWARD) {
                kS += ffStep * 0.1
                updateController()
            }
        }

        Gamepads.gamepad1.square whenBecomesTrue {
            if (currentMode == TuningMode.FEEDFORWARD) {
                kS = (kS - ffStep * 0.1).coerceAtLeast(0.0)
                updateController()
            }
        }

        // ==================== ADJUSTMENT STEP SIZE ====================
        // D-Pad Left/Right to change step size
        Gamepads.gamepad1.dpadLeft whenBecomesTrue {
            pidStep /= 10.0
            ffStep /= 10.0
        }

        Gamepads.gamepad1.dpadRight whenBecomesTrue {
            pidStep *= 10.0
            ffStep *= 10.0
        }

        // ==================== QUICK ACTIONS ====================
        // Start button - Apply and save to RobotConfig
        Gamepads.gamepad1.start whenBecomesTrue {
            RobotConfig.FlywheelConfig.pid = PIDCoefficients(kP, kI, kD)
            RobotConfig.FlywheelConfig.feedforward.kV = kV
            RobotConfig.FlywheelConfig.feedforward.kS = kS
            RobotConfig.FlywheelConfig.feedforward.kA = kA
            gamepad1.rumble(500)
        }

        // Back button - Reset to default
        Gamepads.gamepad1.back whenBecomesTrue {
            kP = 0.08
            kI = 0.0
            kD = 0.005
            kV = 1.0 / 1950.0
            kS = 0.07
            kA = 0.0
            updateController()
            gamepad1.rumble(200)
        }
    }

    private fun updateController() {
        // Recreate the controller with new coefficients
        FlyWheel.controller = dev.nextftc.control.builder.controlSystem {
            velPid(PIDCoefficients(kP, kI, kD))
            basicFF(dev.nextftc.control.feedforward.BasicFeedforwardParameters(kV, kS, kA))
        }
        FlyWheel.controller.goal = KineticState(0.0, targetVelocity)
    }

    override fun onUpdate() {
        // Fine velocity adjustment with left stick Y
        if (abs(gamepad1.left_stick_y) > 0.1) {
            targetVelocity += -gamepad1.left_stick_y.toDouble() * velocityStep * 0.1
            targetVelocity = targetVelocity.coerceIn(0.0, 2000.0)
            FlyWheel.setTargetVelocity(targetVelocity)
        }

        // Display telemetry
        displayTelemetry()
    }

    private fun displayTelemetry() {
        telemetry.clear()

        telemetry.addLine("=== FLYWHEEL PID TUNER ===")
        telemetry.addLine()

        // Current Mode
        telemetry.addData("MODE (DPad ↑↓)", currentMode.name)
        telemetry.addData("Step Size (DPad ←→)", "%.4f".format(if (currentMode == TuningMode.PID) pidStep else ffStep))
        telemetry.addLine()

        // Velocity Info
        telemetry.addLine("--- VELOCITY ---")
        telemetry.addData("Target (L/R Bumper)", "%.0f".format(targetVelocity))
        telemetry.addData("Actual", "%.0f ticks/s".format(FlyWheel.getCurrentVelocity()))
        telemetry.addData("RPM", "%.0f".format(FlyWheel.getRPM()))
        telemetry.addData("Error", "%.0f".format(abs(FlyWheel.getCurrentVelocity() - targetVelocity)))
        telemetry.addData("At Speed", if (FlyWheel.isAtTargetVelocity()) "✓ YES" else "✗ NO")
        telemetry.addLine()

        // PID Coefficients
        telemetry.addLine("--- PID (△ ○ ✕ □) ---")
        telemetry.addData("kP (△/✕)", "%.4f".format(kP))
        telemetry.addData("kI", "%.4f".format(kI))
        telemetry.addData("kD (○/□)", "%.4f".format(kD))
        telemetry.addLine()

        // Feedforward Coefficients
        telemetry.addLine("--- FEEDFORWARD (△ ○ ✕ □) ---")
        telemetry.addData("kV (△/✕)", "%.6f".format(kV))
        telemetry.addData("kS (○/□)", "%.4f".format(kS))
        telemetry.addData("kA", "%.4f".format(kA))
        telemetry.addLine()

        // Controls Help
        telemetry.addLine("--- CONTROLS ---")
        telemetry.addData("L/R Bumper", "Velocity Presets")
        telemetry.addData("Left Stick Y", "Fine Velocity")
        telemetry.addData("DPad ↑↓", "Switch Mode")
        telemetry.addData("DPad ←→", "Step Size")
        telemetry.addData("△ ○ ✕ □", "Tune Values")
        telemetry.addData("START", "Save to Config")
        telemetry.addData("BACK", "Reset to Default")
        telemetry.addLine()

        // Current tuned values (easy to copy)
        telemetry.addLine("--- COPY THESE VALUES ---")
        telemetry.addData("kP", kP)
        telemetry.addData("kI", kI)
        telemetry.addData("kD", kD)
        telemetry.addData("kV", kV)
        telemetry.addData("kS", kS)

        telemetry.update()
    }
}