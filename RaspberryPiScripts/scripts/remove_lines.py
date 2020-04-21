#!/usr/bin/python

import os
import sys

if len(sys.argv) != 2:
	print(str(len(sys.argv)))
	sys.exit("Not enough args")

index = int(sys.argv[1])
lines = 0
datafile = "data_log.csv"
filetopic = ["livingroom/","kitchen/","garage/","outside/"]
filepath = "IoTHub/" + filetopic[index] + datafile

countlines = "wc -l " + filepath + " | awk '{print $1}'" # gets only the total lines back from command

numberoflines = os.popen(countlines).read() # return the files number of lines 

linestoremove = int(numberoflines) - 500 # calculate the number of lines to remove

if linestoremove > 0:
	removelines = "sed -i '2,%id' %s" % (linestoremove, filepath)
	os.system(removelines) # pass command to os.system to operate on the file

