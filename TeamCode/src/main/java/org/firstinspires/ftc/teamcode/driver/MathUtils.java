package org.firstinspires.ftc.teamcode.driver;

import java.lang.Math;

public class MathUtils {
    public static double lerp(double a, double b, double w) { return a * (1.0 - w) + b * w; }
    public static double clamp(double x, double a, double b) { return Math.max(Math.min(x, b), a); }
}
