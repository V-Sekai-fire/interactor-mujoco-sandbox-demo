#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <api.hpp>

// doctest normally owns main(). A sandbox program does not: main() registers
// the guest's API and halts, so the runner is exposed as a call the host makes.
static Variant run_tests() {
	doctest::Context ctx;
	ctx.setOption("no-intro", false);
	ctx.setOption("duration", false);
	const int failed = ctx.run();

	Array out = Array::Create();
	out.push_back(failed);
	return out;
}

int main() {
	ADD_API_FUNCTION(run_tests, "Array", "", "Run the guest test suite, returning the failure count");
	halt();
}
