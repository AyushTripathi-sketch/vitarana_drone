from collections import defaultdict
from Queue import PriorityQueue
adj = defaultdict(list)
pq = PriorityQueue()
dist = [1e5]*no_of_nodes
def djikstras(s):
		dist[s]=0
		vis = [False]*no_of_nodes
		pq.put((dist[s],s))
		while not pq.empty():
			d,u=pq.get()
			if(vis[u]):
				continue
			vis[u]=True
			for v,w in adj[u]:
				if(not vis[v] and dist[u]+w<dist[v]):
					dist[v]=dist[u]+w
					par[v]=u
					pq.put((dist[v],v))