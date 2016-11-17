# Definition of the classes that are present in the dataset
# each class corresponds to a class index. Only add classes with a new, NEVER CHANGE the ORDER
#class2Index = dict(headphones=1, pingpong=2, remotecontrol=3)
#class2Index = dict(book=1, ball=2, bowl=3, halter=4, dove=5)
classes = {
	"i_am_a_bunny_book"                  :  1,
	"laugh_out_loud_joke_book"           :  2,
	"scotch_bubble_mailer"               :  3,
	"up_glucose_bottle"                  :  4,
	"dasani_water_bottle"                :  5,
	"rawlings_baseball"                  :  6,
	"folgers_classic_roast_coffee"       :  7,
	"elmers_washable_no_run_school_glue" :  8,
	"hanes_tube_socks"                   :  9,
	"womens_knit_gloves"                 : 10,
	"cherokee_easy_tee_shirt"            : 11,
	"peva_shower_curtain_liner"          : 12,
	"cloud_b_plush_bear"                 : 13,
	"barkely_hide_bones"                 : 14,
	"kyjen_squeakin_eggs_plush_puppies"  : 15,
	"cool_shot_glue_sticks"              : 16,
	"creativity_chenille_stems"          : 17,
	"soft_white_lightbulb"               : 18,
	"safety_first_outlet_plugs"          : 19,
	"oral_b_toothbrush_red"              : 20,
	"oral_b_toothbrush_green"            : 21,
	"dr_browns_bottle_brush"             : 22,
	"command_hooks"                      : 23,
	"easter_turtle_sippy_cup"            : 24,
	"fiskars_scissors_red"               : 25,
	"scotch_duct_tape"                   : 26,
	"woods_extension_cord"               : 27,
	"platinum_pets_dog_bowl"             : 28,
	"fitness_gear_3lb_dumbbell"          : 29,
	"rolodex_jumbo_pencil_cup"           : 30,
	"clorox_utility_brush"               : 31,
	"kleenex_paper_towels"               : 32,
	"expo_dry_erase_board_eraser"        : 33,
	"kleenex_tissue_box"                 : 34,
	"ticonderoga_12_pencils"             : 35,
	"crayola_24_ct"                      : 36,
	"jane_eyre_dvd"                      : 37,
	"dove_beauty_bar"                    : 38,
	"staples_index_cards"                : 39,
}


classes_short = {
	"bunny book"     : 1,
	"LOL book"       : 2,
	"envelope"       : 3,
	"glucose"        : 4,
	"water"          : 5,
	"baseball"       : 6,
	"coffee"         : 7,
	"glue"           : 8,
	"socks"          : 9,
	"gloves"         : 10,
	"tshirt"         : 11,
	"curtain"        : 12,
	"bear"           : 13,
	"bones"          : 14,
	"squeakin eggs"  : 15,
	"glue sticks"    : 16,
	"chenille stems" : 17,
	"lightbulb"      : 18,
	"outlet plugs"   : 19,
	"toothbrush r"   : 20,
	"toothbrush g"   : 21,
	"bottle brush"   : 22,
	"hooks"          : 23,
	"sippy cup"      : 24,
	"scissors"       : 25,
	"duct tape"      : 26,
	"cord"           : 27,
	"bowl"           : 28,
	"dumbbell"       : 29,
	"pencil cup"     : 30,
	"brush"          : 31,
	"paper roll"     : 32,
	"eraser"         : 33,
	"kleenex"        : 34,
	"pencils"        : 35,
	"crayola"        : 36,
	"dvd"            : 37,
	"dove"           : 38,
	"index cards"    : 39,
}


inverse_classes = {v: k for k, v in classes.items()}
inverse_classes_short = {v: k for k, v in classes_short.items()}


def getShortName(long_name):
	# use short classenames for user input and display
	class_number = classes[long_name]
	short_name = inverse_classes_short[class_number]
	return short_name

def getLongName(short_name):
	# use short classenames for user input and display
	class_number = classes_short[short_name]
	long_name = inverse_classes[class_number]
	return long_name
