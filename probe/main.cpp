#include <api.hpp>
#include <vector>
#include <string>

// No print between fetch and return: separates a faulting fetch from a fetch
// that corrupts the state the next print needs.
static Variant fetch_silent(Variant v) {
	const std::vector<uint8_t> b = v.as_byte_array().fetch();
	return (int)b.size();
}

static Variant fetch_then_print(Variant v) {
	const std::vector<uint8_t> b = v.as_byte_array().fetch();
	print("after fetch");
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

int main() {
	ADD_API_FUNCTION(fetch_silent, "int", "PackedByteArray b", "");
	ADD_API_FUNCTION(fetch_then_print, "int", "PackedByteArray b", "");
	ADD_API_FUNCTION(string_silent, "int", "String s", "");
	ADD_API_FUNCTION(print_twice, "int", "Variant v", "");
	halt();
}
