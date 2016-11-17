#!/usr/bin/python
import sys
import argparse

# Map ROS message types to C++ types.
cpp_types = {
	'bool'   : 'bool',
	'int8'   : 'std::int8_t',
	'uint8'  : 'std::uint8_t',
	'int16'  : 'std::int16_t',
	'uint16' : 'std::uint16_t',
	'int32'  : 'std::int32_t',
	'uint32' : 'std::uint32_t',
	'int64'  : 'std::int64_t',
	'uint64' : 'std::uint64_t',
}

class Enum:
	def __init__(self, name):
		self.name = name
		self.type = None
		self.values = []

	def sanitized(self):
		if not self.values: raise ValueError('Empty enum detected.')
		same_types = all(map(lambda x: x[0] == self.values[0][0], self.values))
		if not same_types: raise ValueError('Mixed type enum detected.')

		sanitized        = Enum(self.name)
		sanitized.type   = self.values[0][0]
		sanitized.values = list(map(lambda x: x[1:], self.values))
		return sanitized

	def __str__(self):
		return "({}: {})".format(self.name, ",".join(map(str, self.values)))

def parseValue(line):
	declaration, value = line.split('=', 1)
	declaration = declaration.strip()
	type, name = declaration.split(None, 1)
	return (type.strip(), name.strip(), value.strip())


def parseDefinitions(blob, name):
	current = None
	for linenum, line in enumerate(blob.splitlines()):
		line = line.strip()

		if not line:
			if current is None: continue
			yield current.sanitized()
			current = None
			continue

		if line.startswith('#:'):
			keyword, name = line[2:].strip().split(None, 1)
			keyword       = keyword.strip()
			name          = name.strip()
			if keyword == 'enum':
				current = Enum(name)
			else:
				raise ValueError("{}:{}: Unknown keyword `{}'".format(name, linenum, keyword))
			continue

		if current is None: continue
		try:
			current.values.append(parseValue(line))
		except Exception as e:
			raise ValueError("{}:{}: {}".format(name, linenum, str(e)))

def lowercaseFirst(string):
	return string[0].lower() + string[1:] if string else ''

def writeCppToStringHeader(output, enum, indent = ''):
	output.write(indent)
	function_prefix = lowercaseFirst(enum.name);
	output.write('std::string {}ToString({} value);\n'.format(function_prefix, cpp_types[enum.type]))

def writeCppFromStringHeader(output, enum, indent = ''):
	output.write(indent)
	function_prefix = lowercaseFirst(enum.name);
	output.write('{} {}FromString(std::string const & value);\n'.format(cpp_types[enum.type], function_prefix))

def writeCppHeader(output, enums, namespace):
	output.write('#pragma once\n')
	output.write('#include <cstdint>\n')
	output.write('#include <string>\n')
	output.write('\n')
	output.write('namespace {} {{\n'.format(namespace))
	output.write('\n')
	for enum in enums:
		writeCppToStringHeader(output, enum)
		writeCppFromStringHeader(output, enum)
	output.write('\n')
	output.write('}\n')


def writeCppToStringSource(output, enum, indent = ''):
	output.write(indent)
	function_prefix = lowercaseFirst(enum.name);
	output.write('std::string {}ToString({} value) {{\n'.format(function_prefix, cpp_types[enum.type]))
	output.write(indent)
	output.write('\tswitch (value) {\n')
	for name, value in enum.values:
		output.write(indent)
		output.write('\t\tcase {}: return "{}";\n'.format(value, name.lower()))
	output.write(indent)
	output.write('\t}\n')
	output.write('\tthrow std::runtime_error("Invalid {} value: " + std::to_string(value) + ".");\n'.format(enum.name))
	output.write(indent)
	output.write('}\n')

def writeCppFromStringSource(output, enum, indent = ''):
	output.write(indent)
	function_prefix = lowercaseFirst(enum.name);
	output.write('{} {}FromString(std::string const & value) {{\n'.format(cpp_types[enum.type], function_prefix))
	for name, value in enum.values:
		output.write(indent)
		output.write('\tif (value == "{}") return {};\n'.format(name.lower(), value))
	output.write(indent)
	output.write('\tthrow std::runtime_error("Invalid {} string value: `" + value + "\'.");\n'.format(enum.name))
	output.write(indent)
	output.write('}\n')

def writeCppSource(output, enums, namespace):
	output.write('#include <cstdint>\n')
	output.write('#include <stdexcept>\n')
	output.write('#include <string>\n')
	output.write('\n')
	output.write('namespace {} {{\n'.format(namespace))
	for enum in enums:
		output.write('\n')
		writeCppToStringSource(output, enum)
		output.write('\n')
		writeCppFromStringSource(output, enum)
	output.write('\n')
	output.write('}\n')

def writePythonToString(output, enum, indent = ''):
	output.write(indent)
	function_prefix = lowercaseFirst(enum.name);
	output.write('{}ToString = {{\n'.format(function_prefix))
	output.write(indent)
	for name, value in enum.values:
		output.write(indent)
		output.write('\t{}: "{}",\n'.format(value, name.lower()))
	output.write(indent)
	output.write('}\n')

def writePythonFromString(output, enum, indent = ''):
	output.write(indent)
	function_prefix = lowercaseFirst(enum.name);
	output.write('{}FromString = {{\n'.format(function_prefix))
	output.write(indent)
	for name, value in enum.values:
		output.write(indent)
		output.write('\t"{}": {},\n'.format(name.lower(), value))
	output.write(indent)
	output.write('}\n')

def writePython(output, enums):
	for enum in enums:
		writePythonToString(output, enum)
		output.write('\n')
		writePythonFromString(output, enum)
	output.write('\n')


def main():
	parser = argparse.ArgumentParser(description='Generate string conversions for enums.')
	parser.add_argument('-v', '--verbose', dest='verbose',       action='store_true',         help='Show verbose output.')
	parser.add_argument('--python',        dest='python',        type=argparse.FileType('w'), help='Write python bindings to the given filename.')
	parser.add_argument('--cxx-source',    dest='cxx_source',    type=argparse.FileType('w'), help='Write a C++ header with function declarations to the given filename.')
	parser.add_argument('--cxx-header',    dest='cxx_header',    type=argparse.FileType('w'), help='Write a C++ source file with function definitions to the given filename.')
	parser.add_argument('--cxx-namespace', dest='cxx_namespace',                              help='The namespace to use for C++ code (required for C++ output).')
	parser.add_argument('files', nargs='+',                      type=argparse.FileType('r'), help='The message definitions to process.')
	options = parser.parse_args()

	# Make sure namespace is set if C++ output is requested.
	if options.cxx_namespace is None and (options.cxx_header is not None or options.cxx_source is not None):
		parser.print_usage(file=sys.stderr)
		sys.stderr.write('{}: C++ namespace is required when generating C++ output.\n'.format(sys.argv[0]))
		return 1

	# Parse input files.
	enums = []
	for file in options.files:
		if options.verbose: sys.stderr.write("Reading definitions from `{}'.\n".format(options.file.name))
		enums += parseDefinitions(file.read(), file.name)
		file.close()

	# Write output.
	if options.python is not None:
		if options.verbose: sys.stderr.write("Writing python code to `{}'.\n".format(options.python.name))
		writePython(options.python, enums)
	if options.cxx_header is not None:
		if options.verbose: sys.stderr.write("Writing C++ header to `{}'.\n".format(options.cxx_header.name))
		writeCppHeader(options.cxx_header, enums, options.cxx_namespace)
	if options.cxx_source is not None:
		if options.verbose: sys.stderr.write("Writing C++ source to `{}'.\n".format(options.cxx_source.name))
		writeCppSource(options.cxx_source, enums, options.cxx_namespace)

	return 0

if __name__ == '__main__': sys.exit(main())
