using MathNet.Numerics.LinearAlgebra;

public abstract class Frame
{
  public double X { get; set; }
  public double Y { get; set; }
  public double Z { get; set; }
  public double Rx { get; set; }
  public double Ry { get; set; }
  public double Rz { get; set; }

  public abstract Matrix<double> M { get; }

  public Frame() { }
  public Frame(string s)
  {
    string[] split = s
      .Replace("{", "")
      .Replace("}", "")
      .Split(" ", StringSplitOptions.RemoveEmptyEntries);

    X = double.Parse(split[0]);
    Y = double.Parse(split[1]);
    Z = double.Parse(split[2]);
    Rx = double.Parse(split[3]);
    Ry = double.Parse(split[4]);
    Rz = double.Parse(split[5]);
  }

  public override string ToString() => $"{X} {Y} {Z} {Rx} {Ry} {Rz}";

  public override bool Equals(object? obj)
  {
    if (obj is not Frame f)
    {
      return false;
    }

    const double tol = 1e-9;

    return Math.Abs(X - f.X) < tol
        && Math.Abs(Y - f.Y) < tol
        && Math.Abs(Z - f.Z) < tol
        && Math.Abs(Rx - f.Rx) < tol
        && Math.Abs(Ry - f.Ry) < tol
        && Math.Abs(Rz - f.Rz) < tol;
  }
}

public class RzXyzRxRyFrame : Frame
{
  public override Matrix<double> M => Matrix.Mz(Rz) * Matrix.T(X, Y, Z) * Matrix.Mx(Rx) * Matrix.My(Ry);

  public RzXyzRxRyFrame() { }
  public RzXyzRxRyFrame(string s) : base(s) { }

  public RzXyzRxRyFrame(Matrix<double> m)
  {
    double m00 = m[0, 0];
    double m01 = m[0, 1];
    double m02 = m[0, 2];
    double m10 = m[1, 0];
    double m11 = m[1, 1];
    double m12 = m[1, 2];
    double m20 = m[2, 0];
    double m21 = m[2, 1];
    double m22 = m[2, 2];
    const double ONE = 0.9999999;

    // RzRxRy order
    Rx = Math.Asin(m21.Clamp(-1, 1)).Degrees();

    if (Math.Abs(m21) < ONE)
    {
      Ry = Math.Atan2(-m20, m22).Degrees();
      Rz = Math.Atan2(-m01, m11).Degrees();
    }
    else
    {
      Ry = 0;
      Rz = Math.Atan2(m10, m00).Degrees();
    }

    var t = Matrix.Mz(Rz).Inverse() * m.T();
    X = t[0, 3] * 1000; // m to mm
    Y = t[1, 3] * 1000; // m to mm
    Z = t[2, 3] * 1000; // m to mm
  }
}

public class XyzRxRyRzFrame : Frame
{
  public override Matrix<double> M => Matrix.T(X, Y, Z) * Matrix.Mx(Rx) * Matrix.My(Ry) * Matrix.Mz(Rz);

  public XyzRxRyRzFrame() { }

  public XyzRxRyRzFrame(Matrix<double> m)
  {
    double m00 = m[0, 0];
    double m01 = m[0, 1];
    double m02 = m[0, 2];
    double m10 = m[1, 0];
    double m11 = m[1, 1];
    double m12 = m[1, 2];
    double m20 = m[2, 0];
    double m21 = m[2, 1];
    double m22 = m[2, 2];
    const double ONE = 0.9999999;

    // RxRyRz order
    Ry = Math.Asin(m02.Clamp(-1, 1)).Degrees();

    if (Math.Abs(m02) < ONE)
    {
      Rx = Math.Atan2(-m12, m22).Degrees();
      Rz = Math.Atan2(-m01, m00).Degrees();
    }
    else
    {
      Rx = Math.Atan2(m21, m11).Degrees();
      Rz = 0;
    }

    var t = m.T();
    X = t[0, 3] * 1000; // m to mm
    Y = t[1, 3] * 1000; // m to mm
    Z = t[2, 3] * 1000; // m to mm
  }
}
