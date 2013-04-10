#!/usr/bin/python3.3

def parser(subsection):
	title = re.search('{(.*)}',subsection)
	title =  title.group()
	subsection = subsection.replace(title,"").strip() 
	title = re.sub('[{}]',"", title)
	return (title, subsection)
	
def itemrep(content):
	content = re.sub(r'\\\\',"\n", content)
	content=content.replace("\\","@")
	content = re.sub(r'@begin{itemize}',"<ul>",content)
	content = re.sub(r'@end{itemize}',"</ul>", content)
	content = re.sub(r'@item',"<li>", content)
	content = re.sub(r'(@textbf[ ]*{)(.*)(})', r'<b>\2</b>', content)
#	content = imgtag(content)
	return content


def imgtag(cont):
#	cont=re.sub(r'\b',"$",cont)
	content=re.findall(r'$egin{figure}',cont)
	content=re.findall(r'{minipage}',cont)
	print(content,'\n')
	for i in content:
#		print(i)
#		continue
		p=[]
		t=[]
		paths = re.findall( 'h]{[A-Za-z0-9_/.]+', i, re.MULTILINE )
		for dirtyLoc in paths:
			loc = "../" + dirtyLoc[3:]
			p.append(loc)
		titles = re.findall( 'on{[A-Za-z0-9()\-]+', i, re.MULTILINE )
		for dirtyTitle in titles:
			label = dirtyTitle[3:]
			t.append(label)
		ste=""
		for i in range(len(p)):
			ste += "<img src=" + p[i] + ", height=\"100pt\>\n"
			ste += "<p>"+ t[i]+"</p>\n"
		cont= cont.replace(i,ste)
	return cont


################################################################################################

import os
import re
import sys

texfile = "./doc/g13_prof_report.tex"
output = "./doc/g13_lab09_report.html"

if os.path.exists( texfile ):
	text = open( texfile, 'r' )
else:
	print( "Chosen texfile was not found... ", end = '' )
	sys.exit()


fpout = open(output,'w')

global line
line = text.read()

startHtml ="<html>\n<head>\n\t<title>g13_lab09_report</title>\n</head>\n \n<body>"
endHtml ="</body>\n</html>" 

fpout.write(startHtml)


SL=[]

sections =  line.split("\section")
for subc in sections[1:] :
	#print(subc)
	title = re.search('{(.*)}',subc)
	title =  title.group()
	subc = subc.replace(title,"")  
	title = re.sub('[{}]',"", title)
	subc = subc.replace(title,"")  
	subsections = subc.split("\subsection")
	content=subsections[0]
	subsecL=[]
	for i in subsections[1:]:
		subsecL.append(parser(i))
	secTuple=(title,content.strip())
	SL.append((secTuple,subsecL))
	

HTML=""
for S in SL:
	HTML+="<h1>"+S[0][0]+"</h1>\n"
	HTML+="<p>" + itemrep(S[0][1])+ "</p\n"
	for sub in S[1]:
		HTML+="<h2>"+ sub[0]+"</h2>\n"
		HTML+="<p>" + itemrep(sub[1]) + "</p\n"

#print(HTML)

fpout.write(HTML)
fpout.write(endHtml)
text.close()
fpout.close()

