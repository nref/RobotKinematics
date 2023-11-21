using System.Collections;

namespace RobotKinematics.Tests
{
  public class JointToFrameTestCaseSource : IEnumerable
  {
    public IEnumerator GetEnumerator()
    {
      yield return new object[]
      {
        new JointControlPoint
        {
          LA = 0,
          A1 = -90,
          A2 = -75,
          A3 = 75,
          A4 = -185,
          A5 = -95,
          A6 = 15,
        },

        new RzXyzRxRyFrame("{2019.399573118808 -309.1081362002429 -94.22313638173519 7.928151208755095 -3.081651737314916 -104.89649513722456}")
      };

      yield return new object[]
      {
        new JointControlPoint
        {
          LA = 0,
          A1 = -90,
          A2 = -75,
          A3 = 75,
          A4 = -185,
          A5 = -95,
          A6 = 0,
        },

        new RzXyzRxRyFrame("{1781.833892780723 -813.187349346094 -101.03002476846434 6.801505340673491 -4.548712458184068 -89.82107736215309}")
      };

      yield return new object[]
      {
        new JointControlPoint
        {
          LA = 0,
          A1 = -90,
          A2 = -75,
          A3 = 75,
          A4 = -185,
          A5 = -90,
          A6 = 0,
        },

        new RzXyzRxRyFrame("{1839.8087875690308 -808.535423206096 -97.94944952066226 6.8101636422805925 0.4676116549786132 -89.97985856477094}")
      };

      yield return new object[]
      {
        new JointControlPoint
        {
          LA = 0,
          A1 = -90,
          A2 = -75,
          A3 = 75,
          A4 = -180,
          A5 = -90,
          A6 = 0,
        },

        new RzXyzRxRyFrame("{1839.830574677345 -751.4835668649415 -125.0747002324285 1.8101639488056298 0.46936797314110557 -89.97999068769958}")
      };

      yield return new object[]
      {
        new JointControlPoint
        {
          LA = 0,
          A1 = -90,
          A2 = -90,
          A3 = 90,
          A4 = -180,
          A5 = -90,
          A6 = 0,
        },

        new RzXyzRxRyFrame("{1542.188690959718 -751.379621867636 -164.25999999999996 1.8101639488056298 0.46936797314110557 -89.97999068769958}")
      };

      yield return new object[]
      {
        // robot start position
        new JointControlPoint
        {
          LA = 0,
          A1 = 0,
          A2 = -90,
          A3 = 90,
          A4 = -180,
          A5 = -90,
          A6 = 0,
        },

        new RzXyzRxRyFrame("{452.5694163553968 339.00097059114086 -164.25999999999996 1.8101639488056298 0.46936797314110557 0.020009312300405334}")
      };
    }
  }
}