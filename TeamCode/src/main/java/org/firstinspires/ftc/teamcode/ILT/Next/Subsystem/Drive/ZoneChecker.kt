package org.firstinspires.ftc.teamcode.robot.subsystems.drive

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import kotlin.math.abs

/**
 * Checks if robot is in valid shooting zones on the field.
 * Zones are defined as triangular regions.
 */
class ZoneChecker(
    private val robotWidth: Double = RobotConfig.ROBOT_WIDTH,
    private val robotLength: Double = RobotConfig.ROBOT_LENGTH
) {

    data class Point(val x: Double, val y: Double)

    // Zone definitions (for RED alliance - will be mirrored for BLUE)
    private val obstacleZone = listOf(
        Point(0.0, 115.0),
        Point(25.0, 144.0),
        Point(0.0, 141.0)
    )

    private val upperShootZone = listOf(
        Point(0.0, 115.0),
        Point(25.0, 144.0),
        Point(72.0, 72.0)
    )

    private val lowerShootZone = listOf(
        Point(48.0, 0.0),
        Point(72.0, 24.0),
        Point(72.0, 0.0)
    )

    /**
     * Check if point is inside triangle using barycentric coordinates.
     */
    private fun pointInTriangle(p: Point, a: Point, b: Point, c: Point): Boolean {
        val det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y)
        if (abs(det) < 1e-6) return false

        val u = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / det
        val v = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / det
        val w = 1 - u - v

        return u >= 0 && v >= 0 && w >= 0
    }

    /**
     * Get robot bounding box corners.
     */
    private fun getRobotCorners(x: Double, y: Double): List<Point> {
        val hw = robotWidth / 2.0
        val hl = robotLength / 2.0
        return listOf(
            Point(x - hw, y - hl),
            Point(x + hw, y - hl),
            Point(x + hw, y + hl),
            Point(x - hw, y + hl)
        )
    }

    /**
     * Check if any corner overlaps with a triangle.
     */
    private fun overlaps(corners: List<Point>, triangle: List<Point>): Boolean {
        return corners.any { pointInTriangle(it, triangle[0], triangle[1], triangle[2]) }
    }

    /**
     * Mirror a zone across field center for BLUE alliance.
     */
    private fun mirrorZone(zone: List<Point>): List<Point> {
        val fieldCenter = RobotConfig.FIELD_SIZE / 2.0
        return zone.map { Point(RobotConfig.FIELD_SIZE - it.x, it.y) }
    }

    /**
     * Check if robot is in a valid shooting zone.
     * Takes alliance into account for zone mirroring.
     */
    fun inShootZone(x: Double, y: Double, alliance: Alliance = RobotConfig.alliance): Boolean {
        val corners = getRobotCorners(x, y)

        // Get zones (mirrored if BLUE alliance)
        val upper = if (alliance == Alliance.RED) upperShootZone else mirrorZone(upperShootZone)
        val lower = if (alliance == Alliance.RED) lowerShootZone else mirrorZone(lowerShootZone)
        val obstacle = if (alliance == Alliance.RED) obstacleZone else mirrorZone(obstacleZone)

        val inUpper = overlaps(corners, upper)
        val inLower = overlaps(corners, lower)
        val inObstacle = overlaps(corners, obstacle)

        return (inUpper || inLower) && !inObstacle
    }
}