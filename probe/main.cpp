#include <api.hpp>
#include <vector>
#include <string>

// Which directions across the sandbox boundary actually work. Guest-to-host is
// the direction the demo depends on; host-to-guest is recorded as faulting.

static Variant fetch_silent(Variant v) {
	const std::vector<uint8_t> b = v.as_byte_array().fetch();
	return (int)b.size();
}

static Variant string_silent(Variant v) {
	const std::string s = v.as_std_string();
	return (int)s.size();
}

static Variant print_twice(Variant v) {
	print("one");
	print("two");
	return 0;
}

// The path mjc_qpos uses: an Array built guest-side and handed back.
static Variant array_return() {
	Array out = Array::Create();
	for (int i = 0; i < 5; i++) {
		out.push_back((double)i * 0.25);
	}
	return out;
}

// The fallback if Array does not cross: a packed array built from a plain
// vector the guest already owns.
static Variant packed_return() {
	std::vector<double> v;
	for (int i = 0; i < 5; i++) {
		v.push_back((double)i * 0.25);
	}
	return PackedArray<double>(v);
}

int main() {
	ADD_API_FUNCTION(fetch_silent, "int", "PackedByteArray b", "");
	ADD_API_FUNCTION(string_silent, "int", "String s", "");
	ADD_API_FUNCTION(print_twice, "int", "Variant v", "");
	ADD_API_FUNCTION(array_return, "Array", "", "");
	ADD_API_FUNCTION(packed_return, "PackedFloat64Array", "", "");
	halt();
}
