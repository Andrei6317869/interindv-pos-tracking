import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({"font.size":8,"font.family":"serif"})
# Fig 1: 5-node example, graph + matrix + degrees
pos={0:(0.1,0.5),1:(0.6,0.95),2:(0.6,0.05),3:(1.1,0.5),4:(1.6,0.5)}
E=[(0,1),(0,2),(0,3),(1,2),(2,3),(3,4)]
n=5;A=np.zeros((n,n),int)
for i,j in E:A[i,j]=A[j,i]=1
fig,(a,b)=plt.subplots(1,2,figsize=(3.45,1.75),gridspec_kw={"width_ratios":[1.25,1]})
for i,j in E:a.plot(*zip(pos[i],pos[j]),"k-",lw=1.2)
for k,(x,y) in pos.items():
    a.scatter([x],[y],s=260,c="white" if k else "black",edgecolors="black",zorder=3,linewidths=1.2,marker="s" if k==0 else "o")
    a.text(x,y,str(k),ha="center",va="center",color="white" if k==0 else "black",zorder=4,weight="bold")
a.set_xlim(-0.1,1.8);a.set_ylim(-0.1,1.1);a.axis("off")
a.set_title("graph (0 = beacon)",fontsize=7.5)
b.axis("off");b.set_title("matrix $A$ and degree $k$",fontsize=7.5)
for i in range(n):
    for j in range(n):
        b.text(j,-i,str(A[i,j]),ha="center",va="center",weight="bold" if A[i,j] else "normal",color="black" if A[i,j] else "0.45")
    b.text(n+0.3,-i,str(A[i].sum()),ha="center",va="center")
    b.text(-0.9,-i,str(i),ha="center",va="center",fontsize=7)
    b.text(i,0.8,str(i),ha="center",va="center",fontsize=7)
b.text(n+0.3,0.8,"$k$",ha="center",va="center",fontsize=7)
b.set_xlim(-1.3,n+0.8);b.set_ylim(-n+0.3,1.3)
plt.tight_layout(pad=0.3);plt.savefig("figures/fig_graph_example_bw.png",dpi=300);plt.close()
# Fig 2: layers
fig,ax=plt.subplots(figsize=(3.45,2.1))
ax.set_xlim(0,10);ax.set_ylim(0,6.4);ax.axis("off")
ax.scatter([5],[5.8],s=160,marker="D",c="white",edgecolors="black",lw=1.2,zorder=3);ax.text(5,5.8,"C",ha="center",va="center",weight="bold",zorder=4)
ax.text(5.5,5.8,"layer 0: coordinator (no edges)",va="center",fontsize=6.5)
G={"g1":(2.8,3.6),"g2":(7.2,3.6)}
ax.plot(*zip(G["g1"],G["g2"]),"k-",lw=1.2);ax.text(5,3.85,"$W_{12}$",ha="center",fontsize=7)
for k,(x,y) in G.items():
    ax.scatter([x],[y],s=520,c="0.8",edgecolors="black",lw=1.4,zorder=3);ax.text(x,y,"G"+k[1],ha="center",va="center",weight="bold",zorder=4)
ax.text(0.1,4.5,"layer 1: groups",fontsize=6.5)
M={"g1":[(1.6,1.3),(2.8,0.7),(4.0,1.3)],"g2":[(6.0,1.3),(7.2,0.7),(8.4,1.3)]}
for k,pts in M.items():
    for x,y in pts:
        ax.plot([G[k][0],x],[G[k][1],y],"k:",lw=0.8)
    for i in range(3):
        for j in range(i+1,3):ax.plot(*zip(pts[i],pts[j]),"k-",lw=0.8)
    for x,y in pts:ax.scatter([x],[y],s=110,c="white",edgecolors="black",lw=1,zorder=3)
ax.plot([4.0,6.0],[1.3,1.3],"k-",lw=1.2)
ax.text(5,1.55,"outside edge",ha="center",fontsize=6)
ax.text(0.1,0.1,"layer 2: people (group members fully linked)",fontsize=6.5)
plt.tight_layout(pad=0.2);plt.savefig("figures/fig_layers_bw.png",dpi=300);plt.close()
