#!/usr/bin/python3

def parser(subsection):
	title = re.search('{(.*)}',subsection)
	title =  title.group()
	subsection = subsection.replace(title,"").strip() 
	title = re.sub('[{}]',"", title)
	return (title, subsection)
	
def itemrep(content):
	content = re.sub(r'\\\\',"\n", content)
	content=content.replace("\\","~")
	content = re.sub(r'~begin[ ]*{itemize}',"\t<ul>",content)
	content = re.sub(r'~end {document}',"",content)
	content = re.sub(r'~end[ ]*{itemize}',"\t</ul>", content)
	content = re.sub(r'~item',"\t\t<li>", content)
	content = re.sub(r'(~textbf[ ]*{)(.*)(})', r'<b>\2</b>', content)
	content = re.sub(r'(~textit[ ]*{)(.*?)(})', r'<i>\2</i>', content,re.DOTALL)
	content = re.sub(r'(~texttt{)(.*?)(})', r'<b>\2</b>', content)
	content = re.sub(r'~_',"_", content)
	content = re.sub(r'~newpage',"", content)
	content = re.sub(r'~noindent{}',"", content)
	#print(content,'------------------------------------','\n')
	
	content = imgtag(content)
	return content


def imgtag(cont):	
	content2=re.findall(r'(~begin{figure})(.*?)(end{figure})',cont,re.DOTALL)
	
	for j in content2:
		
		content=re.findall(r'(includegraphics)(.*?)(caption{)(.*?)(})',j[1],re.DOTALL)
		ste=""
		for i in content:
			p=[]
			t=[]
			paths = re.findall( 'h]{[A-Za-z0-9_/.]+',i[1], re.MULTILINE )
			
			for dirtyLoc in paths:
				loc = "../" + dirtyLoc[3:]
				p.append(loc)
				mm=i[2]+i[3]+i[4]
			titles = re.findall( '(on{[A-Za-z0-9()\- \,]+)(})',mm, re.MULTILINE )
			
			for dirtyTitle in titles:
				label = dirtyTitle[0][3:]
				t.append(label)
			
		
			for k in range(len(t)):
				ste += """<p align=\"center\"><img src=\"""" + p[k] +".png"+ """ " height="400px " \></p>"""
				ste += "<p align=\"center\"><i><b>"+ t[k]+"</b></i></p>\n"
			
		replace = j[0] + j[1] + j[2] 
		cont= cont.replace(replace,ste)
	return cont


################################################################################################

import os
import re
import sys

texfile = "./doc/g13_report.tex"
output = "./doc/g13_report.html"

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

profilingContent=re.findall(r'(\\section {Performance \(Code Profiling\)})(.*?)(\\section)',line,re.DOTALL)
profilingContent= profilingContent[0][0] + profilingContent[0][1]
#print(profilingContent)
SL=[]

sections =  profilingContent.split("\section")
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
	HTML+="\n<h1>"+S[0][0]+"</h1>\n"
	HTML+="\n<p>" + itemrep(S[0][1])+ "\n</p>\n"
	for sub in S[1]:
		HTML+="\n<h2>"+ sub[0]+"</h2>\n"
		HTML+="\t<p>" + itemrep(sub[1]) + "\n\t</p>\n"

#print(HTML)

fpout.write(HTML)
fpout.write(endHtml)
text.close()
fpout.close()



