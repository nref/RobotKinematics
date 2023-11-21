public static class ListExtensions
{
  public static List<T> Append<T>(this List<T> l, IEnumerable<T> ts)
  {
    l.AddRange(ts);
    return l;
  }

  public static List<T> Append<T>(this List<T> l, params T[] ts)
  {
    l.AddRange(ts);
    return l;
  }
}