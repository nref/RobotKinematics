using MathNet.Numerics.LinearAlgebra;

public static class Matrix
{
  /// <summary>
  /// Return a 4x4 homogenous transformation matrix
  /// with the upper-left 3x3 row-major submatrix 
  /// representing a rotation around the X axis.
  /// 
  /// <para/>
  /// Unit must be degrees.
  /// 
  /// <para/>
  /// Output Example: 
  /// 
  /// <para/>
  /// {                       <br/>
  ///    |       |, 0         <br/>
  ///    |   R   |, 0         <br/>
  ///    |       |, 0         <br/>
  ///     0, 0, 0 , 1         <br/>
  /// }
  /// </summary>
  /// <param name="rx">Angle around the X axis in degrees</param>
  public static Matrix<double> Mx(this double rx)
  {
    double rad = rx.Radians();

    double cosx = Math.Cos(rad);
    double sinx = Math.Sin(rad);

    return new[,]
    {
      {1, 0, 0, 0},
      {0, cosx, -sinx, 0},
      {0, sinx, cosx, 0},
      {0, 0, 0, 1}
    }.M();
  }

  /// <summary>
  /// Return a 4x4 homogenous transformation matrix
  /// with the upper-left 3x3 row-major submatrix 
  /// representing a rotation around the Y axis.
  /// 
  /// <para/>
  /// Unit must be degrees.
  /// 
  /// <para/>
  /// Output Example: 
  /// 
  /// <para/>
  /// {                       <br/>
  ///    |       |, 0         <br/>
  ///    |   R   |, 0         <br/>
  ///    |       |, 0         <br/>
  ///     0, 0, 0 , 1         <br/>
  /// }
  /// </summary>
  /// <param name="ry">Angle around the Y axis in degrees</param>
  public static Matrix<double> My(this double ry)
  {
    double rad = ry.Radians();

    double cosy = Math.Cos(rad);
    double siny = Math.Sin(rad);

    return new[,]
    {
      {cosy, 0, siny, 0},
      {0, 1, 0, 0},
      {-siny, 0, cosy, 0},
      {0, 0, 0, 1}
    }.M();
  }

  /// <summary>
  /// Return a 4x4 homogenous transformation matrix
  /// with the upper-left 3x3 row-major submatrix 
  /// representing a rotation around the Z axis.
  /// 
  /// <para/>
  /// Unit must be degrees.
  /// 
  /// <para/>
  /// Output Example: 
  /// 
  /// <para/>
  /// {                       <br/>
  ///    |       |, 0         <br/>
  ///    |   R   |, 0         <br/>
  ///    |       |, 0         <br/>
  ///     0, 0, 0 , 1         <br/>
  /// }
  /// </summary>
  /// <param name="rz">Angle around the Z axis in degrees</param>
  public static Matrix<double> Mz(this double rz)
  {
    double rad = rz.Radians();

    double cosz = Math.Cos(rad);
    double sinz = Math.Sin(rad);

    return new[,]
    {
      {cosz, -sinz, 0, 0},
      {sinz, cosz, 0, 0},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
    }.M();
  }

  /// <summary>
  /// Return a 4x4 homogenous transformation matrix
  /// representing the given translations.
  /// 
  /// <para/>
  /// Input is in millimeter.
  /// 
  /// <para/>
  /// Translations are in the fourth column, e.g.  <br/>
  /// {                                            <br/>
  ///     1, 0, 0, x                               <br/>
  ///     0, 1, 0, y                               <br/>
  ///     0, 0, 1, z                               <br/>
  ///     0, 0, 0, 1                               <br/>
  /// }
  /// </summary>
  public static Matrix<double> T(double x, double y, double z) => Matrix<double>.Build.DenseOfArray(new[,]
  {
    {1, 0, 0, x / 1000}, // mm to m
    {0, 1, 0, y / 1000}, // mm to m
    {0, 0, 1, z / 1000}, // mm to m
    {0, 0, 0, 1}
  });

 public static Matrix<double> T(this Matrix<double> m) => Matrix<double>.Build.DenseOfArray(new[,]
  {
    {1, 0, 0, m[0, 3]},
    {0, 1, 0, m[1, 3]},
    {0, 0, 1, m[2, 3]},
    {0, 0, 0, 1}
  });

  /// <summary>
  /// Convert a row-major 16-element array to a 4x4 matrix
  /// </summary>
  public static Matrix<double> M(this IEnumerable<double> m) => m == default
      ? Identity
      : Matrix<double>.Build.DenseOfRowMajor(4, 4, m);

  /// <summary>
  /// Convert a row-major 2D 16-element array to a 4x4 matrix
  /// </summary>
  public static Matrix<double> M(this double[,] m) => m == default
      ? Identity
      : Matrix<double>.Build.DenseOfArray(m);

  public static Matrix<double> Identity => Matrix<double>.Build.DenseIdentity(4, 4);

  /// <summary>
  /// Relative transform from a to b
  /// </summary>
  public static Matrix<double> To(this Matrix<double> a, Matrix<double> b) => a.Inverse() * b;

  public static double Degrees(this double rad) => rad * 180 / Math.PI;
  public static double Radians(this double deg) => deg * Math.PI / 180;

  /// <summary>
  /// Clamp the given value to the given min and max.
  /// 
  /// <para/>
  /// For example, Clamp(1050, 0, 1000) => 1000 <br/>
  /// For example, Clamp(-50, 0, 1000) => 0 <br/>
  /// </summary>
  public static T Clamp<T>(this T val, T min, T max) where T : IComparable
  {
    if (val.CompareTo(min) < 0) return min;
    if (val.CompareTo(max) > 0) return max;
    return val;
  }
}