import pandas as pd
import csv
col_names = ['Task','Cell_no','Latitude','Longitude','Altitude']
df = pd.read_csv('/home/maut/catkin_ws/src/vitarana_drone/scripts/manifest.csv',names=col_names,header=None)
cell_no = df.loc[:,'Cell_no']
task = df.loc[:,'Task']
lat = df.loc[:,'Latitude']
lon = df.loc[:,'Longitude']
alt = df.loc[:,'Altitude']
cen_del=[18.9998238365,72.000128216]
cen_ret=[18.9999503135,72.000128216]
edges=[]
path=[]
p=20
ret=[]
deli=[]
path_seq=[]
c=0
for i in task:
	if(i==task[1]):
		ret.append(c)
	else:
		deli.append(c)
	c+=1
for i in ret:
	x=((lat[i]-cen_ret[0])*110692.0702932625)**2
	y=((lon[i]-cen_ret[1])*105292.0089353767)**2
	weight=-1*(5+0.1*pow(x+y,0.5)-0.0003*(x+y))
	edges.append([i+2,1,weight])
for i in deli:
	x=((lat[i]-cen_del[0])*110692.0702932625)**2
	y=((lon[i]-cen_del[1])*105292.0089353767)**2
	weight=-1*(5+0.1*pow(x+y,0.5)-0.0003*(x+y))
	edges.append([0,i+2,weight])
for i in deli:
	for j in ret:
		x=((lat[i]-lat[j])*110692.0702932625)**2
		y=((lon[i]-lon[j])*105292.0089353767)**2
		weight=pow(x+y,0.5)
		edges.append([i+2,j+2,weight])
def path_gen(no_of_nodes):
	dist = [1e5]*no_of_nodes
	dist[0]=0
	par=[-1]*no_of_nodes
	for i in range(0,no_of_nodes):
		for u,v,w in edges:
			if(dist[u]+w<dist[v]):
				par[v]=u
				dist[v]=dist[u]+w
	#print(dist)
	x=1
	path.append(x)
	while(x!=0):
		x=par[x]
		path.append(x)
	path.reverse()
	print(path)
	path_seq.append(path)

for i in range(0,9):
	path_gen(20)
	j=0
	for u,v,w in edges:
		if v==path[1]:
			edges[j][2]=10000
		if u==path[1]:
			edges[j][2]=10000
		if u==path[2]:
			edges[j][2]=10000
		j+=1
	path=[]

filename = 'CostCoordinate4.csv'
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
	for a,b,c,d in path_seq:
		b=b-2
		c=c-2
		x=((lat[b]-cen_del[0])*110692.0702932625)**2
		y=((lon[b]-cen_del[1])*105292.0089353767)**2
		cost=5+0.1*pow(x+y,0.5)
		row1= [cell_no[b],lat[b],lon[b],alt[b],cost,pow(x+y,0.5)]
		x=((lat[c]-cen_ret[0])*110692.0702932625)**2
		y=((lon[c]-cen_ret[1])*105292.0089353767)**2
		cost=5+0.1*pow(x+y,0.5)
		row2= [cell_no[c],lat[c],lon[c],alt[c],cost,pow(x+y,0.5)]
		csvwriter.writerow(row1)
		csvwriter.writerow(row2)

filename = 'mat4.csv'
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
	for a,b,c,d in path_seq:
		b=b-2
		c=c-2
		string1 = cell_no[b]
		x1 = 18.9998102845 + (ord(string1[0])-ord('A'))*1.5/110692.0702932625
		y1 = 72.000142461  + (ord(string1[1])-ord('1'))*1.5/(105292.0089353767)
		string2 = cell_no[c]
		x2 = 18.9999367615 + (ord(string2[0])-ord('X'))*1.5/110692.0702932625
		y2 = 72.000142461  + (ord(string2[1])-ord('1'))*1.5/(105292.0089353767)
		row1= [cell_no[b],x1,y1,16.7079808807]
		row2= [cell_no[c],x2,y2,16.95]
		csvwriter.writerow(row1)
		csvwriter.writerow(row2)