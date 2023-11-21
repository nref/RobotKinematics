using NUnit.Framework;

namespace RobotKinematics.Tests
{
  [TestFixture]
  public class KukaRobotFixture
  {
    [TestCaseSource(typeof(JointToFrameTestCaseSource))]
    public void ForwardPosition_IsCorrect(JointControlPoint jcp, Frame expected)
    {
      KukaRobot rob = new();
      Frame actual = rob.ForwardPosition(jcp);

      Assert.AreEqual(expected, actual);
    }
  }
}