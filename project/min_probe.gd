extends SceneTree
func _init():
	var sb = ClassDB.instantiate("Sandbox")
	sb.set("program", load("res://plans/minstep.elf"))
	print("A print_twice   -> ", sb.vmcall("print_twice", 1))
	print("B fetch_silent  -> ", sb.vmcall("fetch_silent", "hello world".to_utf8_buffer()))
	print("C string_silent -> ", sb.vmcall("string_silent", "hello world"))
	print("D fetch+print   -> ", sb.vmcall("fetch_then_print", "hello world".to_utf8_buffer()))
	quit()
