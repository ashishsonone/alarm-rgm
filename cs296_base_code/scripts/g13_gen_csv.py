#!/usr/bin/python3

import subprocess
import re

arg1 = "./bin/cs296_exe_13"
csv = open( './data/lab05_g13_data.csv', 'w' )

for i in range(100): #range( 0, 100 )
	for j in range( 1, 101 ):
	
		lines = subprocess.Popen( [ arg1, str(j) ], stdout = subprocess.PIPE ).communicate()[0]
		numbers = re.findall( '[0-9.]+', lines.decode( "ascii" ), re.MULTILINE )
		
		csv.write ( str(i)+","+str(j)+"," )
		for x in range( 1, 5 ):
			csv.write( numbers[x]+"," )
		csv.write ( numbers[5]+"\n" )
