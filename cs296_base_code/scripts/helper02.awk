#awk -F "\"*,\"*" -f wscript test.csv >output.txt
#awk arrays are like a box of chocolates , they dont print elements in a particular order
#for csv files shud not have spaces o/w for loop wont work with integer indexes(if u know what i mean)
# $1 : rerun , $2 : iteration val

#output :  1.iteation_val | average over rerun 2.step-time | 3.loop time | 4.collisiontime | 5.vel-update | 6.pos-update
{
		step[$2]=step[$2]+$3
		step_sq[$2]=step_sq[$2]+$3*$3
		loop[$2]=loop[$2]+$7
		coll[$2]=coll[$2]+$4
		vel[$2]=vel[$2]+$5
		pos[$2]=pos[$2]+$6
		
		if($2>maxitr)  #no of rows = maxitr , $2 varies from 1 to maxitr
			maxitr=$2
	    if($1>maxrerun)  #get the max rerun value (i.e no of reruns over which avg needs to be taken = maxrerun+1)
			maxrerun=$1
}
END{
    N=maxrerun+1 #vals are averaged over N reruns
	for (x = 1; x <=maxitr ; x++)
		print x","step[x]/N","loop[x]/N","coll[x]/N","vel[x]/N","pos[x]/N","sqrt(((step_sq[x])-(step[x]*step[x]/N))/(N-1))
}
