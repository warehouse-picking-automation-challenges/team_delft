binIndexFromString = {
	"bin_A": 0,
	"bin_B": 1,
	"bin_C": 2,
	"bin_D": 3,
	"bin_E": 4,
	"bin_F": 5,
	"bin_G": 6,
	"bin_H": 7,
	"bin_I": 8,
	"bin_J": 9,
	"bin_K": 10,
	"bin_L": 11,
}

binIndexToString = {
	0:"bin_A",
	1:"bin_B",
	2:"bin_C",
	3:"bin_D",
	4:"bin_E",
	5:"bin_F",
	6:"bin_G",
	7:"bin_H",
	8:"bin_I",
	9:"bin_J",
	10:"bin_K",
	11:"bin_L",
}

def isWide(bin_index):
	return True if bin_index == 1 or bin_index == 4 or bin_index == 7 or bin_index == 10 else False
