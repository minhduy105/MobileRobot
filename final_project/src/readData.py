import csv
import copy
mydict = {}
with open('People.csv', mode='r') as infile:
	reader = csv.reader(infile)
	for rows in reader:
		wayspoint = [rows[1],rows[2],rows[3],rows[4],rows[5],rows[6],rows[7],rows[8]]
		mydict[rows[0]] = wayspoint
	
print (mydict)

print (mydict['Duy'][2])

lit = copy.deepcopy(mydict['Duy'])

print (lit)

a = lit
a.pop(0)
print (a)
print (lit)
print (mydict)