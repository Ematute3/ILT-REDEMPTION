package org.firstinspires.ftc.teamcode.robot.opmodes

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.Gate
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.OuttakeController
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.DriveTrain
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Limelight
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(DriveTrain, FlyWheel, Intake, Turret, Hood, Limelight, OuttakeController, Gate),
            BulkReadComponent, BindingsComponent
        )
    }

    private enum class AimModeTele { OFF, FUSED }
    private var currentMode = AimModeTele.OFF

    override fun onInit() {
        RobotConfig.alliance = Alliance.RED
        RobotState.reset()
        follower.pose = Pose(0.0, 0.0, 0.0)
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            { -gamepad1.left_stick_y.toDouble() },
            { -gamepad1.left_stick_x.toDouble() },
            { -gamepad1.right_stick_x.toDouble() },
            false // FIXED: Set to true (Matches your smooth Test Op)
        ).schedule()

        bindControls()
    }

    private fun bindControls() {
        // --- DRIVER (GP1) ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue Intake.run whenBecomesFalse Intake.stop
        Gamepads.gamepad1.leftBumper whenBecomesTrue Intake.reverse whenBecomesFalse Intake.stop
        Gamepads.gamepad1.rightTrigger.greaterThan(0.5) whenBecomesTrue FlyWheel.spin whenBecomesFalse FlyWheel.stop

        Gamepads.gamepad1.rightBumper whenBecomesTrue ShootCommands.shoot
        Gamepads.gamepad1.circle whenBecomesTrue ShootCommands.stopAll
        Gamepads.gamepad1.triangle whenBecomesTrue { currentMode = AimModeTele.FUSED }
        Gamepads.gamepad1.square whenBecomesTrue { follower.setPose(Pose(follower.pose.x, follower.pose.y, 0.0)) }

        // --- OPERATOR (GP2) ---
        // Gate Controls
        Gamepads.gamepad1.dpadLeft whenBecomesTrue Gate.open
        Gamepads.gamepad1.dpadRight whenBecomesTrue Gate.close

        // Hood Adjustments (Incremental nudges)
        Gamepads.gamepad1.dpadUp whenBecomesTrue Hood.moveUp
        Gamepads.gamepad1.dpadDown whenBecomesTrue Hood.moveDown
    }

    override fun onUpdate() {
        // Sync Odo
        RobotState.currentX = follower.pose.x
        RobotState.currentY = follower.pose.y
        val rawHeading = follower.pose.heading
        RobotState.currentHeading = if (abs(rawHeading) > 2.0 * PI) Math.toRadians(rawHeading) else rawHeading
        RobotState.poseValid = true

        // Turret Logic Execution
        when (currentMode) {
            AimModeTele.OFF -> Turret.stop()
            AimModeTele.FUSED -> Turret.aimWithBoth()
        }

        // --- Manual Overrides ---
        // If the operator touches the sticks, it should probably disable Auto-Aim to prevent fighting
        val tInput = gamepad2.right_stick_x.toDouble()
        if (abs(tInput) > 0.1) {
            currentMode = AimModeTele.OFF
            Turret.currentState = Turret.State.MANUAL
            Turret.manualPower = tInput * RobotConfig.TurretConfig.manualPowerFast
        }

        val hInput = -gamepad2.left_stick_y.toDouble()
        if (abs(hInput) > 0.1) {
            Hood.setPosition(RobotState.hoodPosition + (hInput * 0.01))
        }

        // Telemetry
        telemetry.addData("Mode", currentMode)
        telemetry.addData("Turret Aligned", RobotState.turretAligned)
        telemetry.addData("Hood Pos", "%.2f".format(RobotState.hoodPosition))
        telemetry.addData("Flywheel", if (FlyWheel.isAtTargetVelocity()) "READY" else "SPINNING")
        telemetry.update()
    }
}