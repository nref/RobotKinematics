using MathNet.Numerics.LinearAlgebra;

public class KukaRobot
{
  readonly Matrix<double> m0 = new[]
  {
    1, 0, 0, -1.09,
    0, 1, 0, 0,
    0, 0, 1, 0.98,
    0, 0, 0, 1
  }.M();

  readonly Matrix<double> m1 = new[]
  {
    1,0,0,0.35,
    0,1,0,0,
    0,0,1,-0.675,
    0,0,0,1
  }.M();

  readonly Matrix<double> m2 = new[]
  {
    1,0,0,-0.74,
    0,1,0,0,
    0,0,1,-0.845,
    0,0,0,1
  }.M();

  readonly Matrix<double> m3 = new[]
  {
    1,0,0,.012,
    0,1,0,0,
    0,0,1,-0.804,
    0,0,0,1
  }.M();

  readonly Matrix<double> m4 = new[]
  {
    1,0,0,0.448,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
  }.M();

  readonly Matrix<double> m5 = new[]
  {
    1,0,0,0.46,
    0,1,0,0,
    0,0,1,-0.584,
    0,0,0,1
  }.M();

  /// <summary>
  /// Absolute transform of A1
  /// </summary>
  public Matrix<double> A1 => m0;
  /// <summary>
  /// Absolute transform of A2
  /// </summary>
  public Matrix<double> A2 => m0 * m1;
  /// <summary>
  /// Absolute transform of A3
  /// </summary>
  public Matrix<double> A3 => m2;
  /// <summary>
  /// Absolute transform of A4
  /// </summary>
  public Matrix<double> A4 => m3;
  /// <summary>
  /// Absolute transform of A5
  /// </summary>
  public Matrix<double> A5 => m3 * m4;
  /// <summary>
  /// Absolute transform of A6
  /// </summary>
  public Matrix<double> A6 => m5;

  public Matrix<double> FromA1ToA2 => A1.To(A2);
  public Matrix<double> FromA2ToA3 => A2.To(A3);
  public Matrix<double> FromA3ToA4 => A3.To(A4);
  public Matrix<double> FromA4ToA5 => A4.To(A5);
  public Matrix<double> FromA5ToA6 => A5.To(A6);

  /// <summary>
  /// Translation, a6 to ttcs
  /// </summary>
  Matrix<double> t = Matrix.T(-9.26, 342.78, 402.04);
  /// <summary>
  /// Rx, a6 to ttcs
  /// </summary>
  Matrix<double> rx = Matrix.Mx(1.64);
  /// <summary>
  /// Ry, a6 to ttcs
  /// </summary>
  Matrix<double> ry = Matrix.My(0.29);
  /// <summary>
  /// Rz, a6 to ttcs
  /// </summary>
  Matrix<double> rz = Matrix.Mz(0.21);
  /// <summary>
  /// Relative transform, a6 to ttcs
  /// </summary>
  public Matrix<double> A6ToTtcs => t * rx * ry * rz;

  /// <summary>
  /// FixedSystem values array from manufacturer
  /// 
  /// <c>
  /// = new[] { psi Rx, phi Ry, theta Rz, x, y, z }
  /// </c>
  /// 
  /// <para/>
  /// Values in degree and mm
  /// </summary>
  readonly double[] fs = new[] { 0.10632, 0.0178792, -0.0651333, 3550.21, 30.6245, -812.13 };

  Matrix<double> fsT => Matrix.T(fs[3], fs[4], fs[5]);
  Matrix<double> fsMx => Matrix.Mx(fs[0]);
  Matrix<double> fsMy => Matrix.My(fs[1]);
  Matrix<double> fsMz => Matrix.Mz(fs[2]);

  /// <summary>
  /// Transform, IEC position to robot base.
  /// </summary>
  public Matrix<double> FixedSystem => fsT * fsMx * fsMy * fsMz;

  readonly JointControlPoint start = new()
  {
    LA = 0,
    A1 = 0,
    A2 = -90,
    A3 = 90,
    A4 = -180,
    A5 = -90,
    A6 = 0,
  };

  public JointControlPoint At(JointControlPoint jcp) => new()
  {
    LA = jcp.LA - start.LA,
    A1 = jcp.A1 - start.A1,
    A2 = jcp.A2 - start.A2,
    A3 = jcp.A3 - start.A3,
    A4 = jcp.A4 - start.A4,
    A5 = jcp.A5 - start.A5,
    A6 = jcp.A6 - start.A6,
  };

  /// <summary>
  /// Convert the joint position to a room position, i.e. get
  /// the end of the kinematic chain. Returns an IEC61217 position.
  /// </summary>
  public Frame ForwardPosition(JointControlPoint jcp) 
  {
    List<Matrix<double>> chain = GetChain(At(jcp));
    Matrix<double> m = ForwardPosition(chain);
    return new RzXyzRxRyFrame(m);
  }

  /// <summary>
  /// Get the entire kinematic chain at the given joint position.
  /// </summary>
  public List<Frame> ForwardPositions(JointControlPoint jcp)
  {
    List<Matrix<double>> chain = GetChain(At(jcp));
    List<Matrix<double>> ms = ForwardPositions(chain);
    return ms.Select(m => (Frame)new XyzRxRyRzFrame(m)).ToList();
  }

  public Matrix<double> ForwardPosition(List<Matrix<double>> chain)
  {
    var mout = Matrix.Identity;

    foreach (var m in chain)
    {
      mout = mout * m;
    }
    return mout;
  }

  public List<Matrix<double>> ForwardPositions(List<Matrix<double>> chain)
  {
    var mout = Matrix.Identity;
    List<Matrix<double>> msout = new(chain.Count);

    foreach (var m in chain)
    {
      mout = mout * m;
      msout.Add(mout.Clone());
    }

    return msout;
  }

  public List<Matrix<double>> GetChain(JointControlPoint jcp) => new()
  {
    Matrix.T(jcp.LA, 0, 0),
    // Put A1 at FixedSystem
    A1.Inverse(),
    FixedSystem.Inverse(),
    A1,
    Matrix.Mz(jcp.A1),
    FromA1ToA2,
    Matrix.My(-jcp.A2),
    FromA2ToA3,
    Matrix.My(-jcp.A3),
    FromA3ToA4,
    Matrix.Mx(-jcp.A4),
    FromA4ToA5,
    Matrix.My(jcp.A5),
    FromA5ToA6,
    Matrix.Mz(-jcp.A6),
    A6ToTtcs
  };
}