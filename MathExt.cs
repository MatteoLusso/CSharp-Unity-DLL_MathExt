/*
   ___ ___  ___   ___ ___ ___  _   _ ___    _   _        
  | _ \ _ \/ _ \ / __| __|   \| | | | _ \  /_\ | |       
  |  _/   / (_) | (__| _|| |) | |_| |   / / _ \| |__     
  |_| |_|_\\___/ \___|___|___/ \___/|_|_\/_/ \_\____|    
                                                        
 ___ __  __   _   ___ ___ _  _   _ _____ ___ ___  _  _ 
|_ _|  \/  | /_\ / __|_ _| \| | /_\_   _|_ _/ _ \| \| |
 | || |\/| |/ _ \ (_ || || .` |/ _ \| |  | | (_) | .` |
|___|_|  |_/_/ \_\___|___|_|\_/_/ \_\_| |___\___/|_|\_|
                                                           

                                         MATH EXTENSION

                                   Author: Matteo Lusso
                                                 © 2021

*/

/*
 
This is a DLL for Unity that adds some algebric and geometric functions.

Compile to generate the DLL file.

*/

using System;
using System.Collections.Generic;
using UnityEngine;

namespace DLL_MathExt
{
    public static class Geometry
    {
        public class Ellipse
        {
            protected float width;
            protected float height;

            public Ellipse(float inputWidth, float inputHeight)
            {
                width = inputWidth;
                height = inputHeight;
            }
            public static Vector2 IntersectionWithLine2D(Ellipse elipse, float dirSlope)
            {

                // m2 is the square of target_DirSlope, a2 = a^2 and b2 = b^2.
                float m2 = Mathf.Pow(dirSlope, 2), a2 = Mathf.Pow(elipse.width, 2), b2 = Mathf.Pow(elipse.height, 2);

                // Consider an ellipse centered in the axis origin. That means the screen center is the axis origin, so the line passing throught the screen center coordinates and the target coordinates
                // has the equation [y = mx] while the ellipse has the equation [((x^2)/(a^2) + (y^2)/(b^2)) = 1].
                // After all calculation the coordinate x of the insersection point is [x = √((a² * b²) / ((a² * m²) + b²))] and the y one is [y = tan(α) * x].
                // α is the angle between the Vector3.right and the line passing through the screen center and the target position and tan(α) = target_DirSlope.
                float x = Mathf.Sqrt((a2 * b2) / ((a2 * m2) + b2));
                float y = dirSlope * x;

                return new Vector2(x, y);
            }
        }
    }

    public static class Algebra
    {
        // Use this function to lerp a value between a repeated range.
        //
        //                ├───────┬─────────────────────────────────┼───────┬─────────────────────────┬───────┼───────┬─────────────────────────────────┤
        //  Min - (Max - Min)     │                                Min      │                         │      Max      │                                Max + (Max - Min)
        //                      B - (Max - Min)                             B                         A             B + (Max - Min)
        //
        //                                                                  └─────────────────────────┘ |B - A| = D
        //
        //                        └───────────────────────────────────────────────────────────────────┘ |(B - (Max - Min)) - A| = DMinus
        //
        //                                                                                            └───────────────┘ |(B + (Max - Min)) - A| = DPlus
        //
        //In this example, the range goes from the Min and Max values.If A = 0.9, and B = 0.1, the common Mathf.Lerp function reduces the A value to reach the B float.
        //ShortestLerp instead checks what value between D, DMinus, and DPlus is the lower one.In the example, DPlus<D<DMinus, so the A increases to the Max value and, 
        //when it becomes higher (A >= Max), it returns to Min value and continues to rise till it reaches B.
        public static float ShortestLerp(float A, float B, float Min, float Max, float t)
        {
            float R = (Max - Min);
            float D = Mathf.Abs(B - A);
            float DMinus = Mathf.Abs(B - R - A);
            float DPlus = Mathf.Abs(B + R - A);

            if (D <= DMinus && D <= DPlus)
            {
                A = Mathf.Lerp(A, B, t);
            }
            else if (DMinus >= DPlus)
            {
                A = Mathf.Lerp(A, B + R, t);
            }
            else
            {
                A = Mathf.Lerp(A, B - R, t);
            }

            if (A <= Min)
            {
                A += R;
            }
            else if (A >= Max)
            {
                A -= R;
            }

            return A;
        }

        // It returns the module [√(Re² + Im²)] of a complex conjugate number [Re ± j * Im].
        public static float Module(float Re, float Im)
        {
            return Mathf.Sqrt(Mathf.Pow(Re, 2) + Mathf.Pow(Im, 2));
        }

        // This methods return the equivalent value in a different range.
        // For example, if A = 50, RangeA(0, 100) and RangeB(100, -100), it returns 0.
        //
        //   ├────────────┬────────────┤       <->       ├────────────┬────────────┤
        //   0         A = 50        100                100         B = 0        -100
        public static float EquivalentValueInDifferentRange(float A, Vector2 RangeA, Vector2 RangeB)
        {
            return ((RangeB.y - RangeB.x) * (A - RangeA.x) / (RangeA.y - RangeA.x)) + RangeB.x;
        }

        // This function returns the determinat of a 2x2 matrix. 
        //
        //         ┌─    ─┐        ┌─              ─┐
        //         │ Row1 │        │ Row1.x  Row1.y │
        //         │      │    =   │                │ 
        //         │ Row2 │        │ Row2.x  Row2.y │
        //         └─    ─┘        └─              ─┘
        public static float Determinant2x2(Vector2 Row1, Vector2 Row2)
        {
            return (Row1.x * Row2.y) - (Row2.x * Row1.y);
        }

        // Given two Vector3(A, B, C) with the coefficients A, B, C of two 2D lines [A * x + B * y = C], this function return the
        // coordinates of the intersection point between the two lines, if it exists.
        public static Vector2 TwoLinesIntersection2D(Vector3 CoeffLine1, Vector3 CoeffLine2) // A * x + B * y = C -> A, B, and C are the coefficents of a line.
        {
            float Determinant = Determinant2x2(CoeffLine1, CoeffLine2);

            if (Determinant != 0.0f)
            {
                float XCoord = ((CoeffLine2.y * CoeffLine1.z) - (CoeffLine1.y * CoeffLine2.z)) / Determinant;
                float YCoord = ((CoeffLine1.x * CoeffLine2.z) - (CoeffLine2.x * CoeffLine1.z)) / Determinant;

                return new Vector2(XCoord, YCoord);
            }
            else
            {
                throw new ArgumentException("The two lines are parallel.");
            }
        }

        // This method, given two points coordinates (x and y), gives back a Vector3 with the coefficients A, B and C of the
        // equation of the line (A * x + B * y = C) passing throught the two points.
        public static Vector3 LinePassingThroughTwoPoints2D(Vector2 PointA, Vector2 PointB)
        {
            float A = PointB.y - PointA.y;
            float B = PointA.x - PointB.x;
            float C = (A * PointA.x) + (B * PointA.y);

            return new Vector3(A, B, C);    // -> Vector3{x, y, z} <-> {A, B, C}
        }

        // This method, given three points coordinates (x, y and z), gives back a Vector4 with the coefficients A, B, C and D of the
        // equation of the plane (A * x + B * y + C * z = D) where the three points lie.
        public static Vector4 PlanePassingThroughThreePoints3D(Vector3 PointA, Vector3 PointB, Vector3 PointC)
        {
            float A1 = PointB.x - PointA.x;
            float B1 = PointB.y - PointA.y;
            float C1 = PointB.z - PointA.z;
            float A2 = PointC.x - PointA.x;
            float B2 = PointC.y - PointA.y;
            float C2 = PointC.z - PointA.z;
            float A = B1 * C2 - B2 * C1;
            float B = A2 * C1 - A1 * C2;
            float C = A1 * B2 - B1 * A2;
            float D = ((-A * PointA.x) + (-B * PointA.y) + (-C * PointA.z));

            return new Vector4(A, B, C, D); // -> Vector4{x, y, z, w} <-> {A, B, C, D}
        }
    }
    public static class Angles
    {
        // Given an angle in degrees, this function converts it to a signed angle included between -180 and 180.
        // For example, 1035° -> -45°.
        public static float Repeat180(float Angle)
        {
            float RepeatedAngle = Angle;

            float halfRounds;

            if (Angle < -180.0f)
            {
                halfRounds = (int)(Angle / -180.0f);

                if ((int)halfRounds % 2 == 0)
                {
                    RepeatedAngle = ((int)halfRounds * 180.0f) + Angle;
                }
                else
                {
                    RepeatedAngle = (1.0f - halfRounds + (int)halfRounds) * 180.0f;
                }
            }
            else if(Angle > 180.0f)
            {
                halfRounds = (int)(Angle / 180.0f);

                if ((int)halfRounds % 2 == 0)
                {
                    RepeatedAngle = Angle - ((int)halfRounds * 180.0f);
                }
                else
                {
                    RepeatedAngle = (1.0f - halfRounds + (int)halfRounds) * -180.0f;
                }
            }

            return RepeatedAngle;
        }

        // This function transforms an angle that goes from -180 to 180 degrees, to a positive angle included between 0 and 360.
        public static float SignedAngleTo360Angle(float SignedAngle)
        {
            SignedAngle = Repeat180(SignedAngle);

            if (SignedAngle < 0.0f)
            {
                SignedAngle = 360.0f + SignedAngle;
            }
            return SignedAngle;
        }
    }
}
