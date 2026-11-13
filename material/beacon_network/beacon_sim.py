import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import ListedColormap
from collections import deque

plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.linewidth":0.8})
N, R, D, SEP = 18, 1.0, 1.9, 0.32   # agents, radio range, scatter radius, min spacing

def build(seed):
    rng = np.random.default_rng(seed); pts=[np.zeros(2)]
    while len(pts)<N+1:
        r=D*np.sqrt(rng.uniform()); t=rng.uniform(0,2*np.pi); p=np.array([r*np.cos(t),r*np.sin(t)])
        if all(np.linalg.norm(p-q_)>SEP for q_ in pts): pts.append(p)
    P=np.array(pts); d=np.linalg.norm(P[:,None]-P[None],axis=2)
    A=((d<=R)&(d>0)).astype(int)
    hops=np.full(N+1,-1); hops[0]=0; q=deque([0])
    while q:
        u=q.popleft()
        for v in np.where(A[u])[0]:
            if hops[v]<0: hops[v]=hops[u]+1; q.append(v)
    return P,A,hops

for SEED in range(200):
    P,A,hops=build(SEED)
    if A[0].sum()==6 and (hops>=0).all() and hops.max()==3: break
# order agents by hop count so the matrix shows structure
order=[0]+sorted(range(1,N+1),key=lambda i:(hops[i],np.linalg.norm(P[i])))
P=P[order]; A=A[np.ix_(order,order)]; hops=hops[order]

C = {0:"#C0392B",1:"#1F6F8B",2:"#E0A030",3:"#6C5B7B",-1:"#9AA5B1"}
lab = {1:"1 hop (direct)",2:"2 hops",3:"3 hops",-1:"Unreachable"}
def col(h): return C[h] if h in C else "#6C5B7B"

fig,(ax,bx)=plt.subplots(1,2,figsize=(12,5.6),gridspec_kw={"width_ratios":[1.25,1]},dpi=200)
fig.patch.set_facecolor("white")

# --- spatial graph
ax.set_aspect("equal"); ax.set_facecolor("#F7F9FB")
ax.add_patch(Circle((0,0),R,fc="#C0392B",alpha=0.07,ec="#C0392B",lw=1.2,ls="--",zorder=1))
for i in range(N+1):
    for j in range(i+1,N+1):
        if A[i,j]:
            b = (i==0)
            ax.plot(*P[[i,j]].T,color="#C0392B" if b else "#7F8C9A",
                    lw=1.5 if b else 1.0,alpha=0.75 if b else 0.55,zorder=2)
for i in range(1,N+1):
    ax.scatter(*P[i],s=170,c=col(hops[i]),ec="white",lw=1.5,zorder=4)
    ax.text(*P[i],str(i),ha="center",va="center",fontsize=8,color="white",fontweight="bold",zorder=5)
ax.scatter(0,0,s=420,marker="*",c=C[0],ec="white",lw=1.2,zorder=5)
ax.text(0.0,-0.2,"Beacon",fontsize=9,color=C[0],fontweight="bold",ha="center",va="top",zorder=6)
ax.text(R*np.cos(np.radians(125))-0.05,R*np.sin(np.radians(125))+0.06,"range $R$",fontsize=9,
        color=C[0],ha="right",va="bottom",style="italic")
L=D+0.3; ax.set_xlim(-L,L); ax.set_ylim(-L,L)
ax.set_xlabel("$x/R$"); ax.set_ylabel("$y/R$")
for s in ("top","right"): ax.spines[s].set_visible(False)
ax.grid(color="white",lw=1.2)
h=[plt.Line2D([],[],marker="o",ls="",ms=9,color=C[k],label=lab[k]) for k in (1,2,3,-1) if (hops==k).any()]
ax.legend(handles=h,loc="upper left",frameon=False,fontsize=9)
ax.set_title("(a) Agents scattered around a beacon",loc="left",fontweight="bold",fontsize=11)

# --- adjacency matrix
bx.imshow(A,cmap=ListedColormap(["#F2F4F7","#1F3A5F"]),vmin=0,vmax=1)
bx.set_xticks(range(N+1)); bx.set_yticks(range(N+1))
names=["B"]+[str(i) for i in range(1,N+1)]
bx.set_xticklabels(names,fontsize=8); bx.set_yticklabels(names,fontsize=8)
bx.xaxis.tick_top(); bx.tick_params(length=0)
bx.set_xticks(np.arange(-.5,N+1,1),minor=True); bx.set_yticks(np.arange(-.5,N+1,1),minor=True)
bx.grid(which="minor",color="white",lw=1.2); bx.tick_params(which="minor",length=0)
for s in bx.spines.values(): s.set_visible(False)
for k in range(N+1):
    bx.get_xticklabels()[k].set_color(col(hops[k])); bx.get_yticklabels()[k].set_color(col(hops[k]))

bx.set_title("(b) Adjacency matrix $A$ (sorted by hop count)",loc="left",fontweight="bold",fontsize=11,pad=28)

fig.suptitle(f"Short-range wireless network: $N={N}$ agents, uniform in disc of radius {D}$R$, seed {SEED}",
             y=0.99,fontsize=12)
fig.tight_layout(rect=(0,0,1,0.96))
fig.savefig("/home/claude/sim/beacon_network.png",bbox_inches="tight")
fig.savefig("/home/claude/sim/beacon_network.pdf",bbox_inches="tight")

print("seed",SEED,"beacon degree:",A[0].sum()," hops:",hops.tolist())
print("A symmetric:",(A==A.T).all()," edges:",A.sum()//2)
