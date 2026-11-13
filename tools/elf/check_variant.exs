defmodule CheckVariant do
  def main(argv) do
    {kv, pairs, _} = OptionParser.parse(argv, strict: [plant: :boolean])
    pairs == [] && fail("usage: check_variant.exs [--plant] <elf>=<24|40> ...")

    Enum.each(pairs, fn pair ->
      [path, size] = String.split(pair, "=")
      check(path, String.to_integer(size), kv[:plant] || false)
    end)
  end

  defp check(path, want, plant) do
    want in [24, 40] || fail("#{path}: the size is 24 or 40, not #{want}")
    bin = File.read!(path)
    match?(<<0x7F, "ELF", 2, 1, _::binary>>, bin) || fail("#{path}: not a little-endian ELF64")
    <<_::binary-size(16), type::little-16, machine::little-16, _::binary>> = bin
    type == 2 || fail("#{path}: not ET_EXEC (#{type})")
    machine == 243 || fail("#{path}: not EM_RISCV (#{machine})")
    found = for {at, _} <- :binary.matches(bin, "SBXV"), do: :binary.decode_unsigned(binary_part(bin, at + 4, 4), :little)
    found = if plant, do: Enum.map(found, &(64 - &1)), else: found
    found == [want] || fail("#{path}: wanted one SBXV marker of size #{want}, found #{inspect(found, charlists: :as_lists)}")
    IO.puts("OK #{path}: ET_EXEC EM_RISCV, SBXV size #{want}")
  end

  defp fail(msg) do
    IO.puts(:stderr, "check_variant: " <> msg)
    System.halt(1)
  end
end

CheckVariant.main(System.argv())
