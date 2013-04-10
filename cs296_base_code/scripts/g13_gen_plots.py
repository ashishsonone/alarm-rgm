#!/usr/bin/python3

from pylab import *
import math

csvdatafile="./data/lab05_g13_data.csv"
ROLLNO=5

##DEFINITIONS START----------------------------------------------------------------------------------

def ripbyl(X,Y,f):
					#both X range from 1 to some integer
					# X is the x values(eg Itr values)
					#Y is the actual data

					#f is the function to be the list of each itr val
	D={}			# here D is the dictionary will store the list of data points corr ot each itr val
	maxX=0
	for i in range(len(X)):
		t=X[i]
		if t not in D.keys():
			D[t]=[Y[i]]
		else:
			D[t]+=[Y[i]]
		if(t>maxX):maxX=t
	#make a list to return
	# print(f([3,2]))
	# print(D[1])
	L=[]
	for i in range(int(maxX)):   #i varies from 0 to t-1 but X values are from 1 to t
		# print(D[i+1])
		L.append(f(D[i+1]))
	return L

#----------------------DEFINITIONS DONE ------------------------------------------------------------#

# ROLLNO=5
#read data from the csv file
data = genfromtxt('./data/lab05_g13_data.csv', delimiter=',')

rrN=data[:,0]
itr=data[:,1]
stepT=data[:,2]
collT=data[:,3]
velT=data[:,4]
posT=data[:,5]
loopT=data[:,6]

#first for  average over reruns  :: first argument should be itr
stepL=ripbyl(itr,stepT,lambda x:np.mean(x))
collL=ripbyl(itr,collT,lambda x:np.mean(x))
velL=ripbyl(itr,velT,lambda x:np.mean(x))
posL=ripbyl(itr,posT,lambda x:np.mean(x))
loopL=ripbyl(itr,loopT,lambda x:np.mean(x))
steperror=ripbyl(itr,stepT,lambda x:np.std(x))
itrL=list(range(1,(len(stepL)+1)))

# print(itrL)

#PLOT1 -- looptime and step time vs itrval
figure(1)
plot(itrL,stepL,color="red",linewidth=1.2,linestyle="-",label="Step-Time")
plot(itrL,loopL,color="blue",linewidth=1.2,linestyle="-",label="Loop Time")
xlabel("Iteration Value")
ylabel("Time (ms)")
legend(loc=7)
title("Step-Time & Loop Time (Averaged over Reruns) vs Iteration Value")
savefig('./plots/g13_lab09_plot01.png')

#PLOT2 -- step and other times vs itrval
figure(2)
plot(itrL,collL,color="blue",linewidth=1.2,linestyle="-",label="Collision Time")
plot(itrL,stepL,color="red",linewidth=1.2,linestyle="-",label="Step-Time")
plot(itrL,velL,color="green",linewidth=1.2,linestyle="-",label="Velocity Update Time")
plot(itrL,posL,color="yellow",linewidth=1.2,linestyle="-",label="Position Update Time")
xlabel("Iteration Value")
ylabel("Time (ms)")
legend(loc=7)
title("Step, Collision, Velocity & Position Update \nTimes (Averaged over Reruns) vs Iteration Value")
savefig('./plots/g13_lab09_plot02.png')

#PLOT5 -- step time with error bars vs iteration value
figure(5)
errorbar(itrL,stepL, yerr= steperror, color="blue", linewidth=1.2, linestyle="-", label="Step-Time")
legend(loc=7)
xlabel("Iteration Value")
ylabel("Time (ms)")
title("Step-Time (Average) with Error-Bars (Std. Deviation) vs Iteration Value")
savefig('./plots/g13_lab09_plot05.png')

#PLOT6
step_filter=[] #filter step times corr to itr val =ROLLNO
for i in range(len(itr)):#look in the origing step time list
	if(itr[i]==ROLLNO):
		step_filter.append(stepT[i])

num_bars = 40
counts, b_edges = np.histogram(step_filter, bins=num_bars, normed=False)
cdf = np.cumsum(counts)  # converts [1,3,2] --> [1,4,6]  i.e the points needed to be plot for cumulative graph

figure(6)
hist(step_filter,bins=num_bars,cumulative=False,color="blue",rwidth=0.9,label="Step-Time")
plot(b_edges[1:], cdf,color="red",linewidth=1.3,linestyle="-",label="Cumulative Time")
legend(loc=2,shadow=True)  #2 is upper left position
xlabel("Variation in Step-Time")
ylabel("No. of Observations")
title("Frequency Plot of Step-Time (Variation) for Iteration Value = 05")
savefig('./plots/g13_lab09_plot06.png')

#NOW FOR AVG OVER ITERATIONS ( PLOTS 3 AND 4) overwriting previous lists
stepL=ripbyl(rrN,stepT,lambda x:np.mean(x)) #just pass rrN for value of X
collL=ripbyl(rrN,collT,lambda x:np.mean(x))
velL=ripbyl(rrN,velT,lambda x:np.mean(x))
posL=ripbyl(rrN,posT,lambda x:np.mean(x))
loopL=ripbyl(rrN,loopT,lambda x:np.mean(x))
# steperror=ripbyl(itr,stepT,lambda x:np.std(x))
rerunL=list(range((len(stepL))))

#PLOT3  -- step ,loop time vs reruns
figure(3)
plot(rerunL,stepL,color="red",linewidth=1.2,linestyle="-",label="Step-Time")
plot(rerunL,loopL,color="blue",linewidth=1.2,linestyle="-",label="Loop Time")
xlabel("Rerun Value")
ylabel("Time (ms)")
legend(loc=7)
title("Step-Time & Loop Time (Averaged over Iteration Values) vs Rerun Number")
savefig('./plots/g13_lab09_plot03.png')

#PLOT4 -- step and other times vs itrval
figure(4)
plot(rerunL,collL,color="blue",linewidth=1.2,linestyle="-",label="Collision Time")
plot(rerunL,stepL,color="red",linewidth=1.2,linestyle="-",label="Step-Time")
plot(rerunL,velL,color="green",linewidth=1.2,linestyle="-",label="Velocity Update Time")
plot(rerunL,posL,color="yellow",linewidth=1.2,linestyle="-",label="Position Update Time")
xlabel("Iteration Value")
ylabel("Time (ms)")
legend(loc=7)
title("Step, Collision, Velocity & Position Update \nTimes (Averaged over Iteration Values) vs Rerun Number")
savefig('./plots/g13_lab09_plot04.png')
