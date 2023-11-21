using MathNet.Numerics.LinearAlgebra;

var notStart = new JointControlPoint
{
  LA = 100,
  A1 = -90,
  A2 = -75,
  A3 = 75,
  A4 = -185,
  A5 = -95,
  A6 = 5,
};

// robot start position
var start = new JointControlPoint
{
  LA = 0,
  A1 = 0,
  A2 = -90,
  A3 = 90,
  A4 = -180,
  A5 = -90,
  A6 = 0,
};

JointControlPoint underTest = notStart;

KukaRobot rob = new();
Frame frame = rob.ForwardPosition(underTest);
List<Frame> frames = rob.ForwardPositions(underTest);
List<Matrix<double>> chain = rob.GetChain(rob.At(underTest));
var chainResults = rob.ForwardPositions(chain);

var chainStrings = chain.Select(m => $"{m:F12}");
var chainResultStrings = chainResults.Select(m => $"{m:F12}");

Console.WriteLine($"A1:\n{rob!.A1}");
Console.WriteLine($"A2:\n{rob!.A2}");
Console.WriteLine($"A3:\n{rob!.A3}");
Console.WriteLine($"A4:\n{rob!.A4}");
Console.WriteLine($"A5:\n{rob!.A5}");
Console.WriteLine($"A6:\n{rob!.A6}");
Console.WriteLine($"A1 to A2:\n{rob!.FromA1ToA2}");
Console.WriteLine($"A2 to A3:\n{rob!.FromA2ToA3}");
Console.WriteLine($"A3 to A4:\n{rob!.FromA3ToA4}");
Console.WriteLine($"A4 to A5:\n{rob!.FromA4ToA5}");
Console.WriteLine($"A5 to A6:\n{rob!.FromA5ToA6}");
Console.WriteLine($"A6 to ttcs:\n{rob!.A6ToTtcs}");
Console.WriteLine($"{nameof(KukaRobot.FixedSystem)}:\n{rob!.FixedSystem:F12}");
Console.WriteLine($"ttcs: {chainResults.Last():F12}");

System.Diagnostics.Debugger.Break();
