defmodule Vendor do
  @out "third_party/ms_human_700"

  def main([src, kept]) do
    xmls = reach(src, "MS-Human-700.xml", []) |> Enum.uniq() |> Enum.sort()
    meshes = kept |> File.read!() |> String.split("\n", trim: true) |> Enum.sort()

    for rel <- ["LICENSE" | xmls] do
      dst = Path.join(@out, rel)
      File.mkdir_p!(Path.dirname(dst))
      File.cp!(Path.join(src, rel), dst)
    end

    usdas =
      for rel <- meshes do
        {points, faces} = read_mesh(Path.join(src, rel))
        out = Path.rootname(rel) <> ".usda"
        dst = Path.join(@out, out)
        File.mkdir_p!(Path.dirname(dst))
        File.write!(dst, usda(Path.basename(rel, Path.extname(rel)), Path.basename(rel), points, faces))
        out
      end

    File.write!(Path.join(@out, "FILES.txt"), Enum.join(xmls ++ usdas, "\n") <> "\n")
    IO.puts("vendored #{length(xmls)} XML files and #{length(usdas)} meshes into #{@out}")
  end

  def main(_), do: IO.puts(:stderr, "usage: vendor.exs <ms_human_700> <kept-meshes.txt>") || System.halt(1)

  defp reach(src, rel, seen) do
    if rel in seen do
      seen
    else
      text = File.read!(Path.join(src, rel))

      Regex.scan(~r/<include\s+file="([^"]+)"/, text)
      |> Enum.reduce([rel | seen], fn [_, inc], acc ->
        reach(src, Path.relative_to(Path.expand(Path.join(Path.dirname(rel), inc), "/"), "/"), acc)
      end)
    end
  end

  defp read_mesh(path) do
    case Path.extname(path) do
      ".stl" -> read_stl(File.read!(path))
      ".obj" -> read_obj(File.read!(path))
    end
  end

  defp read_stl(<<_::binary-size(80), count::little-32, rest::binary>>) do
    true = byte_size(rest) == count * 50

    {index, faces} =
      for <<_normal::binary-size(12), corners::binary-size(36), _attr::16 <- rest>>, reduce: {%{}, []} do
        {index, faces} ->
          {index, face} =
            for <<corner::binary-size(12) <- corners>>, reduce: {index, []} do
              {index, face} ->
                index = Map.put_new(index, corner, map_size(index))
                {index, [index[corner] | face]}
            end

          {index, [Enum.reverse(face) | faces]}
      end

    points =
      index
      |> Enum.sort_by(&elem(&1, 1))
      |> Enum.map(fn {<<x::little-float-32, y::little-float-32, z::little-float-32>>, _} -> [x, y, z] end)

    {points, Enum.reverse(faces)}
  end

  defp read_obj(text) do
    lines = String.split(text, "\n", trim: true)
    all = for "v " <> rest <- lines, do: rest |> String.split() |> Enum.take(3) |> Enum.map(&number/1)
    all = List.to_tuple(all)

    first =
      lines
      |> Enum.drop_while(&(not String.starts_with?(&1, "o ")))
      |> Enum.drop(1)
      |> Enum.take_while(&(not String.starts_with?(&1, "o ")))

    polygons =
      for "f " <> rest <- first do
        corners = rest |> String.split() |> Enum.map(&(&1 |> String.split("/") |> hd() |> String.to_integer()))
        true = Enum.all?(corners, &(&1 >= 1))
        Enum.map(corners, &(&1 - 1))
      end

    used = polygons |> List.flatten() |> Enum.uniq() |> Enum.sort()
    slot = used |> Enum.with_index() |> Map.new()
    points = Enum.map(used, &elem(all, &1))

    faces =
      Enum.flat_map(polygons, fn
        [a, b, c] -> [[a, b, c]]
        [a, b, c, d] -> if dist(all, a, c) <= dist(all, b, d), do: [[a, b, c], [a, c, d]], else: [[a, b, d], [b, c, d]]
        [first | others] -> for [b, c] <- Enum.chunk_every(others, 2, 1, :discard), do: [first, b, c]
      end)

    {points, Enum.map(faces, fn face -> Enum.map(face, &slot[&1]) end)}
  end

  defp dist(all, i, j) do
    Enum.zip_with(elem(all, i), elem(all, j), fn a, b -> (a - b) * (a - b) end) |> Enum.sum()
  end

  defp number(text) do
    case Float.parse(text) do
      {value, ""} -> value
    end
  end

  defp usda(name, source, points, faces) do
    [
      "#usda 1.0\n(\n    defaultPrim = \"", name, "\"\n",
      "    doc = \"", source, " of MS-Human-700 (MuJoCo Menagerie, Apache-2.0), as written by tools/human/vendor.exs\"\n",
      "    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n\ndef Mesh \"", name, "\"\n{\n",
      "    int[] faceVertexCounts = [", faces |> Enum.map(fn _ -> "3" end) |> Enum.join(", "), "]\n",
      "    int[] faceVertexIndices = [", faces |> List.flatten() |> Enum.join(", "), "]\n",
      "    point3f[] points = [",
      points |> Enum.map(fn p -> "(" <> Enum.map_join(p, ", ", &Float.to_string/1) <> ")" end) |> Enum.join(", "),
      "]\n    uniform token subdivisionScheme = \"none\"\n}\n"
    ]
  end
end

Vendor.main(System.argv())
