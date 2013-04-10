#awk -F "\"*,\"*" -f wscript test.csv >output.txt
#awk arrays are like a box of chocolates , they dont print elements in a particular order
#for csv files shud not have spaces o/w for loop wont work with integer indexes(if u know what i mean)

# $1 : rerun , $2 : iteration val

#input  :  1.rerun no  | 2.itr_val | 3.step time | 4.coll-time | 5.vel-time | 6.pos-time | 7.loop-time
#output :  1.rerun_val | average over iterations 2.step-time | 3.loop time | 4.collisiontime | 5.vel-update | 6.pos-update
{
		step[$1]=step[$1]+$3 #for rerun no $1
		loop[$1]=loop[$1]+$7
	    coll[$1]=coll[$1]+$4
		vel[$1]=vel[$1]+$5
		pos[$1]=pos[$1]+$6
		
		if($2>maxitr)  #get the max iteration value (no of iterations over which avg needs to be taken = maxitr )
			maxitr=$2
	    if($1>maxrerun)  #no of rows = maxrerun+1 ... $1 varies from 0 to maxrerun
			maxrerun=$1  
}
END{
	for (x = 0; x <maxrerun+1 ; x++)
		print x","step[x]/maxitr","loop[x]/maxitr","coll[x]/maxitr","vel[x]/maxitr","pos[x]/maxitr
}
