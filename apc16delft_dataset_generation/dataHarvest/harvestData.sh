#!/bin/bash

echo 'Usage: bash harvestData.sh -s ${path-2-source} -d ${path-2-DRapc2016}'

# Parse argument
while getopts ":s:d:" opt; do
	case $opt in
		s) srcDir=$OPTARG;;
		d) destDir=$OPTARG;;
	esac
done

CLASSES=("i_am_a_bunny_book"             
	"laugh_out_loud_joke_book"           
	"scotch_bubble_mailer"               
	"up_glucose_bottle"                  
	"dasani_water_bottle"                
	"rawlings_baseball"                  
	"folgers_classic_roast_coffee"       
	"elmers_washable_no_run_school_glue" 
	"hanes_tube_socks"                   
	"womens_knit_gloves"                 
	"cherokee_easy_tee_shirt"            
	"peva_shower_curtain_liner"          
	"cloud_b_plush_bear"                 
	"barkely_hide_bones"                 
	"kyjen_squeakin_eggs_plush_puppies"  
	"cool_shot_glue_sticks"              
	"creativity_chenille_stems"          
	"soft_white_lightbulb"               
	"safety_first_outlet_plugs"          
	"oral_b_toothbrush_red"              
	"oral_b_toothbrush_green"            
	"dr_browns_bottle_brush"             
	"command_hooks"                      
	"easter_turtle_sippy_cup"            
	"fiskars_scissors_red"               
	"scotch_duct_tape"                   
	"woods_extension_cord"               
	"platinum_pets_dog_bowl"             
	"fitness_gear_3lb_dumbbell"          
	"rolodex_jumbo_pencil_cup"           
	"clorox_utility_brush"               
	"kleenex_paper_towels"               
	"expo_dry_erase_board_eraser"        
	"kleenex_tissue_box"                 
	"ticonderoga_12_pencils"             
	"crayola_24_ct"                      
	"jane_eyre_dvd"                      
	"dove_beauty_bar"                    
	"staples_index_cards"                
)
echo 'Harvest Dataset from:' $srcDir
echo 'Save Dataset to:' $destDir/DRapc2016/

# Parse Objects
OBJECTS=()
for dir in $srcDir/*
do
if [ -d "$dir" ]; then
	OBJ="${dir##*/}"
	if [[ ${CLASSES[*]} =~ $OBJ ]]; then
		OBJECTS+=($OBJ)
	fi
fi
done

echo 'Objects to harvest:' ${OBJECTS[*]}

# Harvest!
ELEMENTS=${#OBJECTS[@]}
for ((i=0;i<$ELEMENTS;i++)); do
	# harvest foreground images
	foreground="$destDir/DRapc2016/APC2016/Originals/${OBJECTS[i]}/foreground"
	if [ ! -d $foreground ]; then
		mkdir -p "$foreground"
	fi
	if [ -d $srcDir/${OBJECTS[i]} ]; then
		cp $srcDir/${OBJECTS[i]}/*.png "$foreground"
		echo ${OBJECTS[i]} "foreground: successfully copied $foreground"
	else
		echo $srcDir/${OBJECTS[i]} ' does not exist'
	fi
	# harvest background images
	background="$destDir/DRapc2016/APC2016/Originals/${OBJECTS[i]}/background"
	if [ ! -d $background ]; then
		mkdir -p "$background"
	fi
	if [ -d $srcDir/'background' ]; then
		cp $srcDir/'background'/*.png "$background"
		echo ${OBJECTS[i]} "background: successfully copied $background"
	else
		echo $srcDir/background 'does not exist'
	fi
done

echo 'Harvest done.'

