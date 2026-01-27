package org.firstinspires.ftc.teamcode.ILT.Next

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import kotlinx.coroutines.delay

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.AutoAim.AutoAimCalculator
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.AutoAim.AutoAimCommands
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.IntakeState
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.OuttakeController
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision.Limelight
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.PI
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@TeleOp(name = "Auto-Aim TeleOp (Fixed)", group = "Competition")
class AutoAimTeleOp : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                DriveTrain, Limelight, Turret, FlyWheel,
                Hood, Intake, Gate, OuttakeController
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    private enum class ShooterMode { IDLE, MANUAL_AIM, AUTO_AIM, PREPARE_SHOOT, SHOOTING }
    private var currentMode = ShooterMode.IDLE
    private var showDebugTelemetry = false

    override fun onInit() {
        RobotConfig.alliance = Alliance.RED
        RobotState.reset()
        follower.pose = Pose(0.0, 0.0, 0.0)

        AutoAimCalculator.hoodOffset = 0.06
        AutoAimCalculator.velocityOffset = 100.0
        AutoAimCalculator.preferredMethod = AutoAimCalculator.LookupMethod.AUTO
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            { -gamepad1.left_stick_y.toDouble() },
            { -gamepad1.left_stick_x.toDouble() },
            { -gamepad1.right_stick_x.toDouble() },
            true // Enabled Field-Centric for smoother aiming
        ).schedule()

        bindControls()
    }

    private fun bindControls() {
        // --- INTAKE LOGIC (Fixed to prevent stop-fighting) ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue Intake.run
        Gamepads.gamepad1.leftBumper whenBecomesTrue Intake.reverse

        // Stop only when both are released
        button{ gamepad1.left_trigger <= 0.5f && !gamepad1.left_bumper }
            .whenTrue(Intake.stop)

        // --- SHOOTING MODES ---
        Gamepads.gamepad1.triangle whenBecomesTrue {
            currentMode = ShooterMode.AUTO_AIM
            AutoAimCommands.continuousAim.schedule()
        }

        Gamepads.gamepad1.circle whenBecomesTrue prepareToShoot()

        Gamepads.gamepad1.rightBumper whenBecomesTrue fullAutoShoot()

        Gamepads.gamepad1.rightTrigger.greaterThan(0.5) whenBecomesTrue FlyWheel.spin whenBecomesFalse FlyWheel.stop

        Gamepads.gamepad1.square whenBecomesTrue emergencyStop()

        Gamepads.gamepad1.cross whenBecomesTrue {
            currentMode = ShooterMode.MANUAL_AIM
            AutoAimCommands.continuousAim.cancel()
            Turret.stop()
        }

        // --- RESET HEADING ---
        Gamepads.gamepad1.options whenBecomesTrue {
            follower.setPose(Pose(follower.pose.x, follower.pose.y, 0.0))
        }

        // --- OPERATOR (GP2) ---
        Gamepads.gamepad2.dpadLeft whenBecomesTrue Gate.open
        Gamepads.gamepad2.dpadRight whenBecomesTrue Gate.close

        // Tuning commands use pre-built InstantCommands
        Gamepads.gamepad2.leftBumper whenBecomesTrue AutoAimCommands.adjustHoodOffset(0.01)
        Gamepads.gamepad2.leftTrigger.greaterThan(0.5) whenBecomesTrue AutoAimCommands.adjustHoodOffset(-0.01)

        Gamepads.gamepad2.share whenBecomesTrue { showDebugTelemetry = !showDebugTelemetry }
    }

    private fun prepareToShoot(): Command = SequentialGroup(
        InstantCommand {
            currentMode = ShooterMode.PREPARE_SHOOT
            AutoAimCommands.continuousAim.schedule() // Background scheduling to avoid ParallelGroup deadlock
        },
        WaitUntil { RobotState.flywheelAtSpeed && RobotState.turretAligned }, Delay(3.seconds),
        InstantCommand { /* Ready indicator or rumble could go here */ }
    )

    private fun fullAutoShoot(): Command = SequentialGroup(
        InstantCommand {
            currentMode = ShooterMode.SHOOTING
            AutoAimCommands.continuousAim.schedule()
            FlyWheel.setTargetVelocity(AutoAimCalculator.getFlywheelVelocity())
        },
        // Wait for ready state without blocking the entire command engine indefinitely
        WaitUntil { RobotState.flywheelAtSpeed && RobotState.turretAligned }, Delay(2.5.seconds),
        Delay(100.milliseconds),
        InstantCommand { RobotState.intakeState = IntakeState.FEEDING },
        Delay(600.milliseconds),
        InstantCommand {
            RobotState.intakeState = IntakeState.STOPPED
            FlyWheel.setTargetVelocity(0.0)
            AutoAimCommands.continuousAim.cancel()
            currentMode = ShooterMode.IDLE
        }
    )

    private fun emergencyStop(): Command = InstantCommand {
        currentMode = ShooterMode.IDLE
        FlyWheel.setTargetVelocity(0.0)
        Turret.stop()
        Intake.stop.schedule()
        AutoAimCommands.continuousAim.cancel()
        RobotState.intakeState = IntakeState.STOPPED
    }

    override fun onUpdate() {
        // Sync Data to RobotState
        RobotState.currentX = follower.pose.x
        RobotState.currentY = follower.pose.y
        val rawHeading = follower.pose.heading
        RobotState.currentHeading = if (abs(rawHeading) > 2.0 * PI) Math.toRadians(rawHeading) else rawHeading
        RobotState.poseValid = true

        // Manual Hood Adjustment (Moved to update for smooth movement)
        if (gamepad2.dpad_up) {
            val nextPos = (RobotState.hoodPosition + 0.005).coerceIn(0.0, 1.0)
            Hood.setPosition(nextPos)
        }
        if (gamepad2.dpad_down) {
            val nextPos = (RobotState.hoodPosition - 0.005).coerceIn(0.0, 1.0)
            Hood.setPosition(nextPos)
        }

        if (showDebugTelemetry) showDetailedTelemetry() else showSimpleTelemetry()
    }

    private fun showSimpleTelemetry() {
        joinedTelemetry.addData("Mode", currentMode.name)
        joinedTelemetry.addData("Ready", RobotState.flywheelAtSpeed && RobotState.turretAligned)
        joinedTelemetry.addData("Dist", "%.1f".format(AutoAimCalculator.calculateBestAvailable().distance))
        joinedTelemetry.addData("Pose X", "%.1f".format(RobotState.currentX))
        joinedTelemetry.addData("Pose Y", "%.1f".format(RobotState.currentY))
        joinedTelemetry.addData("Turret Yaw", "%.1f°".format(Math.toDegrees(RobotState.turretYaw)))
        joinedTelemetry.update()
    }

    private fun showDetailedTelemetry() {
        joinedTelemetry.addLine(AutoAimCalculator.getDebugInfo())
        joinedTelemetry.addData("Flywheel Vel", "%.0f".format(RobotState.flywheelVelocity))
        joinedTelemetry.addData("Target Vel", "%.0f".format(RobotState.targetFlywheelVelocity))
        joinedTelemetry.addData("LL Has Target", RobotState.limelightHasTarget)
        joinedTelemetry.addData("LL TX", "%.2f".format(RobotState.limelightTx))
        joinedTelemetry.update()
    }
}