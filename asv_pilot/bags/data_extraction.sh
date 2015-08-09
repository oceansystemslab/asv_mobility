#!/bin/bash

# script for data extraction
DIR=$1
PWD=$(pwd)

# topics names
POSITION_REQ="/emily/pilot/position_req"
UNPACKER_PERF="/modem/unpacker/status"

### script below ###
echo $DIR

if [[ ! -n $DIR ]]
then
	echo "Please provide a directory"
	exit 0
fi

# change dir
cd $DIR

for BAG in $(ls | grep .bag)
do
  echo "Processing $BAG ..."
  PREFIX=$(echo $BAG | cut -d '.' -f 1)
  
	rostopic echo -p -b $BAG /emily/nav/nav_sts > "${PREFIX}_nav_sts.csv"
	rostopic echo -p -b $BAG $POSITION_REQ > "${PREFIX}_position_req.csv"
	rostopic echo -p -b $BAG $UNPACKER_PERF > "${PREFIX}_unpacker_perf.csv"

  
done

exit 0
